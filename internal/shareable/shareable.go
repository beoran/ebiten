// Copyright 2018 The Ebiten Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package shareable

import (
	"fmt"
	"image/color"
	"runtime"
	"sync"

	"github.com/hajimehoshi/ebiten/internal/affine"
	"github.com/hajimehoshi/ebiten/internal/driver"
	"github.com/hajimehoshi/ebiten/internal/graphics"
	"github.com/hajimehoshi/ebiten/internal/hooks"
	"github.com/hajimehoshi/ebiten/internal/packing"
	"github.com/hajimehoshi/ebiten/internal/restorable"
)

var graphicsDriver driver.Graphics

func SetGraphicsDriver(graphics driver.Graphics) {
	graphicsDriver = graphics
}

var (
	minSize = 0
	maxSize = 0
)

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}

func init() {
	hooks.AppendHookOnBeforeUpdate(func() error {
		backendsM.Lock()
		defer backendsM.Unlock()

		resolveDeferred()
		makeImagesShared()
		return nil
	})
}

func resolveDeferred() {
	for _, f := range deferred {
		f()
	}
	deferred = nil
}

// MaxCountForShare represents the time duration when the image can become shared.
//
// This value is exported for testing.
const MaxCountForShare = 10

func makeImagesShared() {
	for i := range imagesToMakeShared {
		i.nonUpdatedCount++
		if i.nonUpdatedCount >= MaxCountForShare {
			i.makeShared()
		}
		delete(imagesToMakeShared, i)
	}
}

func MakeImagesSharedForTesting() {
	makeImagesShared()
}

type backend struct {
	restorable *restorable.Image

	// If page is nil, the backend is not shared.
	page *packing.Page
}

func (b *backend) TryAlloc(width, height int) (*packing.Node, bool) {
	// If the region is allocated without any extension, it's fine.
	if n := b.page.Alloc(width, height); n != nil {
		return n, true
	}

	// Simulate the extending the page and calculate the appropriate page size.
	page := b.page.Clone()
	nExtended := 0
	for {
		if !page.Extend() {
			// The page can't be extended any more. Return as failure.
			return nil, false
		}
		nExtended++
		if n := page.Alloc(width, height); n != nil {
			// The page is just for emulation, so we don't have to free it.
			break
		}
	}

	for i := 0; i < nExtended; i++ {
		b.page.Extend()
	}
	s := b.page.Size()
	b.restorable = b.restorable.Extend(s, s)

	n := b.page.Alloc(width, height)
	if n == nil {
		panic("shareable: Alloc result must not be nil at TryAlloc")
	}
	return n, true
}

var (
	// backendsM is a mutex for critical sections of the backend and packing.Node objects.
	backendsM sync.Mutex

	initOnce sync.Once

	// theBackends is a set of actually shared images.
	theBackends = []*backend{}

	imagesToMakeShared = map[*Image]struct{}{}

	deferred []func()
)

func init() {
	// Lock the mutex before a frame begins.
	//
	// In each frame, restoring images and resolving images happen respectively:
	//
	//   [Restore -> Resolve] -> [Restore -> Resolve] -> ...
	//
	// Between each frame, any image operations are not permitted, or stale images would remain when restoring
	// (#913).
	backendsM.Lock()
}

type Image struct {
	width    int
	height   int
	disposed bool
	screen   bool

	backend *backend

	node *packing.Node

	// nonUpdatedCount represents how long the image is kept not modified with DrawTriangles.
	// In the current implementation, if an image is being modified by DrawTriangles, the image is separated from
	// a shared (restorable) image by ensureNotShared.
	//
	// nonUpdatedCount is increased every frame if the image is not modified, or set to 0 if the image is
	// modified.
	//
	// ReplacePixels doesn't affect this value since ReplacePixels can be done on shared images.
	nonUpdatedCount int

	neverShared bool
}

func (i *Image) moveTo(dst *Image) {
	dst.dispose(false)
	*dst = *i

	// i is no longer available but Dispose must not be called
	// since i and dst have the same values like node.
	runtime.SetFinalizer(i, nil)
}

func (i *Image) isShared() bool {
	return i.node != nil
}

func (i *Image) IsSharedForTesting() bool {
	backendsM.Lock()
	defer backendsM.Unlock()
	return i.isShared()
}

type vertexPutterWithoutLock struct {
	*Image
}

func (i vertexPutterWithoutLock) PutVertex(dst []float32, dx, dy, sx, sy float32, bx0, by0, bx1, by1 float32, cr, cg, cb, ca float32) {
	i.putVertex(dst, dx, dy, sx, sy, bx0, by0, bx1, by1, cr, cg, cb, ca)
}

func (i *Image) ensureNotShared() {
	if i.backend == nil {
		i.allocate(false)
		return
	}

	if !i.isShared() {
		return
	}

	_, _, w, h := i.region()
	newImg := restorable.NewImage(w, h)
	vs := make([]float32, 4*graphics.VertexFloatNum)
	graphics.PutQuadVertices(vs, vertexPutterWithoutLock{i}, 0, 0, w, h, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1)
	is := graphics.QuadIndices()
	newImg.DrawTriangles(i.backend.restorable, vs, is, nil, driver.CompositeModeCopy, driver.FilterNearest, driver.AddressClampToZero)

	i.dispose(false)
	i.backend = &backend{
		restorable: newImg,
	}
}

func (i *Image) makeShared() {
	if i.backend == nil {
		i.allocate(true)
		return
	}

	if i.isShared() {
		return
	}

	if !i.shareable() {
		panic("shareable: makeShared cannot be called on a non-shareable image")
	}

	newI := NewImage(i.width, i.height)
	pixels := make([]byte, 4*i.width*i.height)
	for y := 0; y < i.height; y++ {
		for x := 0; x < i.width; x++ {
			r, g, b, a := i.at(x, y)
			pixels[4*(x+i.width*y)] = r
			pixels[4*(x+i.width*y)+1] = g
			pixels[4*(x+i.width*y)+2] = b
			pixels[4*(x+i.width*y)+3] = a
		}
	}
	newI.replacePixels(pixels)
	newI.moveTo(i)
	i.nonUpdatedCount = 0
}

func (i *Image) region() (x, y, width, height int) {
	if i.backend == nil {
		panic("shareable: backend must not be nil: not allocated yet?")
	}
	if !i.isShared() {
		w, h := i.backend.restorable.Size()
		return 0, 0, w, h
	}
	return i.node.Region()
}

func (i *Image) Size() (width, height int) {
	return i.width, i.height
}

// PutVertices puts the given dst with vertices that can be passed to DrawTriangles.
func (i *Image) PutVertex(dst []float32, dx, dy, sx, sy float32, bx0, by0, bx1, by1 float32, cr, cg, cb, ca float32) {
	backendsM.Lock()
	defer backendsM.Unlock()
	i.putVertex(dst, dx, dy, sx, sy, bx0, by0, bx1, by1, cr, cg, cb, ca)
}

func (i *Image) putVertex(dst []float32, dx, dy, sx, sy float32, bx0, by0, bx1, by1 float32, cr, cg, cb, ca float32) {
	if i.backend == nil {
		i.allocate(true)
	}

	ox, oy, _, _ := i.region()
	oxf, oyf := float32(ox), float32(oy)
	i.backend.restorable.PutVertex(dst, dx, dy, sx+oxf, sy+oyf, bx0+oxf, by0+oyf, bx1+oxf, by1+oyf, cr, cg, cb, ca)
}

func (i *Image) DrawTriangles(img *Image, vertices []float32, indices []uint16, colorm *affine.ColorM, mode driver.CompositeMode, filter driver.Filter, address driver.Address) {
	backendsM.Lock()
	defer backendsM.Unlock()

	if img.disposed {
		panic("shareable: the drawing source image must not be disposed (DrawTriangles)")
	}
	if i.disposed {
		panic("shareable: the drawing target image must not be disposed (DrawTriangles)")
	}
	if img.backend == nil {
		img.allocate(true)
	}

	i.ensureNotShared()

	// Compare i and img after ensuring i is not shared, or
	// i and img might share the same texture even though i != img.
	if i.backend.restorable == img.backend.restorable {
		panic("shareable: Image.DrawTriangles: img must be different from the receiver")
	}

	i.backend.restorable.DrawTriangles(img.backend.restorable, vertices, indices, colorm, mode, filter, address)

	i.nonUpdatedCount = 0
	delete(imagesToMakeShared, i)

	if !img.isShared() && img.shareable() {
		imagesToMakeShared[img] = struct{}{}
	}
}

func (i *Image) Fill(clr color.Color) {
	backendsM.Lock()
	defer backendsM.Unlock()

	if i.disposed {
		panic("shareable: the drawing target image must not be disposed (Fill)")
	}
	if i.backend == nil {
		if _, _, _, a := clr.RGBA(); a == 0 {
			return
		}
	}

	i.ensureNotShared()

	// As *restorable.Image is an independent image, it is fine to fill the entire image.
	i.backend.restorable.Fill(clr)

	i.nonUpdatedCount = 0
	delete(imagesToMakeShared, i)
}

// ClearFramebuffer clears the image with a color. This affects not only the (0, 0)-(width, height) region but also
// the whole framebuffer region.
func (i *Image) ClearFramebuffer() {
	backendsM.Lock()
	defer backendsM.Unlock()
	if i.disposed {
		panic("shareable: the drawing target image must not be disposed (Fill)")
	}
	i.ensureNotShared()

	i.backend.restorable.Clear()
}

func (i *Image) ReplacePixels(p []byte) {
	backendsM.Lock()
	defer backendsM.Unlock()
	i.replacePixels(p)
}

func (i *Image) replacePixels(p []byte) {
	if i.disposed {
		panic("shareable: the image must not be disposed at replacePixels")
	}
	if i.backend == nil {
		if p == nil {
			return
		}
		i.allocate(true)
	}

	x, y, w, h := i.region()
	if p != nil {
		if l := 4 * w * h; len(p) != l {
			panic(fmt.Sprintf("shareable: len(p) must be %d but %d", l, len(p)))
		}
	}
	i.backend.restorable.ReplacePixels(p, x, y, w, h)
}

func (i *Image) At(x, y int) (byte, byte, byte, byte) {
	backendsM.Lock()
	defer backendsM.Unlock()
	return i.at(x, y)
}

func (i *Image) at(x, y int) (byte, byte, byte, byte) {
	if i.backend == nil {
		return 0, 0, 0, 0
	}

	ox, oy, w, h := i.region()
	if x < 0 || y < 0 || x >= w || y >= h {
		return 0, 0, 0, 0
	}

	return i.backend.restorable.At(x+ox, y+oy)
}

// disposeFromFinalizer disposes images, but the actual operation is deferred.
// disposeFromFinalizer is called from finalizers.
//
// A function from finalizer must not be blocked, but disposing operation can be blocked.
// Defer this operation until it becomes safe. (#913)
func (i *Image) disposeFromFinalizer() {
	// deferred doesn't have to be, and should not be protected by a mutex.
	deferred = append(deferred, func() {
		i.dispose(true)
	})
}

func (i *Image) Dispose() {
	backendsM.Lock()
	defer backendsM.Unlock()

	i.dispose(true)
}

func (i *Image) dispose(markDisposed bool) {
	defer func() {
		if markDisposed {
			i.disposed = true
		}
		i.backend = nil
		i.node = nil
		if markDisposed {
			runtime.SetFinalizer(i, nil)
		}
	}()

	if i.disposed {
		return
	}

	if i.backend == nil {
		// Not allocated yet.
		return
	}

	if !i.isShared() {
		i.backend.restorable.Dispose()
		return
	}

	i.backend.page.Free(i.node)
	if !i.backend.page.IsEmpty() {
		// As this part can be reused, this should be cleared explicitly.
		i.backend.restorable.ClearPixels(i.region())
		return
	}

	i.backend.restorable.Dispose()
	index := -1
	for idx, sh := range theBackends {
		if sh == i.backend {
			index = idx
			break
		}
	}
	if index == -1 {
		panic("shareable: backend not found at an image being disposed")
	}
	theBackends = append(theBackends[:index], theBackends[index+1:]...)
}

func (i *Image) IsVolatile() bool {
	backendsM.Lock()
	defer backendsM.Unlock()
	if i.backend == nil {
		// Not allocated yet. Only non-volatile images can do lazy allocation so far.
		return false
	}
	return i.backend.restorable.IsVolatile()
}

func NewImage(width, height int) *Image {
	// Actual allocation is done lazily, and the lock is not needed.
	return &Image{
		width:  width,
		height: height,
	}
}

func (i *Image) shareable() bool {
	if minSize == 0 || maxSize == 0 {
		panic("shareable: minSize or maxSize must be initialized")
	}
	if i.neverShared {
		return false
	}
	return i.width <= maxSize && i.height <= maxSize
}

func (i *Image) allocate(shareable bool) {
	if i.backend != nil {
		panic("shareable: the image is already allocated")
	}

	if i.screen {
		i.backend = &backend{
			restorable: restorable.NewScreenFramebufferImage(i.width, i.height),
		}
		runtime.SetFinalizer(i, (*Image).disposeFromFinalizer)
		return
	}

	if !shareable || !i.shareable() {
		i.backend = &backend{
			restorable: restorable.NewImage(i.width, i.height),
		}
		runtime.SetFinalizer(i, (*Image).disposeFromFinalizer)
		return
	}

	for _, b := range theBackends {
		if n, ok := b.TryAlloc(i.width, i.height); ok {
			i.backend = b
			i.node = n
			runtime.SetFinalizer(i, (*Image).disposeFromFinalizer)
			return
		}
	}
	size := minSize
	for i.width > size || i.height > size {
		if size == maxSize {
			panic(fmt.Sprintf("shareable: the image being shared is too big: width: %d, height: %d", i.width, i.height))
		}
		size *= 2
	}

	b := &backend{
		restorable: restorable.NewImage(size, size),
		page:       packing.NewPage(size, maxSize),
	}
	theBackends = append(theBackends, b)

	n := b.page.Alloc(i.width, i.height)
	if n == nil {
		panic("shareable: Alloc result must not be nil at allocate")
	}
	i.backend = b
	i.node = n
	runtime.SetFinalizer(i, (*Image).disposeFromFinalizer)
}

func (i *Image) MakeVolatile() {
	backendsM.Lock()
	defer backendsM.Unlock()

	i.ensureNotShared()
	i.backend.restorable.MakeVolatile()
	i.neverShared = true
}

func (i *Image) Dump(path string) error {
	backendsM.Lock()
	defer backendsM.Unlock()

	return i.backend.restorable.Dump(path)
}

func NewScreenFramebufferImage(width, height int) *Image {
	// Actual allocation is done lazily.
	i := &Image{
		width:       width,
		height:      height,
		screen:      true,
		neverShared: true,
	}
	return i
}

func EndFrame() error {
	backendsM.Lock()

	restorable.ResolveStaleImages()
	return restorable.Error()
}

func BeginFrame() error {
	defer backendsM.Unlock()

	var err error
	initOnce.Do(func() {
		err = restorable.InitializeGraphicsDriverState()
		if err != nil {
			return
		}
		if len(theBackends) != 0 {
			panic("shareable: all the images must be not-shared before the game starts")
		}
		if graphicsDriver.HasHighPrecisionFloat() {
			minSize = 1024
			// Use 4096 as a maximum size whatever size the graphics driver accepts. There are
			// not enough evidences that bigger textures works correctly.
			maxSize = min(4096, graphicsDriver.MaxImageSize())
		} else {
			minSize = 512
			maxSize = 512
		}
	})
	if err != nil {
		return err
	}

	return restorable.RestoreIfNeeded()
}

func DumpImages(dir string) error {
	backendsM.Lock()
	defer backendsM.Unlock()
	return restorable.DumpImages(dir)
}
