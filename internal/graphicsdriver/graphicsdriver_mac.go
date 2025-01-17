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

// +build darwin,!ios
// +build !js

package graphicsdriver

// #cgo CFLAGS: -x objective-c
// #cgo LDFLAGS: -framework Foundation
//
// #import <Foundation/Foundation.h>
//
// static int getMacOSMinorVersion() {
//   NSOperatingSystemVersion version = [[NSProcessInfo processInfo] operatingSystemVersion];
//   return (int)version.minorVersion;
// }
import "C"

import (
	"sync"

	"github.com/hajimehoshi/ebiten/internal/driver"
	"github.com/hajimehoshi/ebiten/internal/graphicsdriver/metal"
	"github.com/hajimehoshi/ebiten/internal/graphicsdriver/metal/mtl"
	"github.com/hajimehoshi/ebiten/internal/graphicsdriver/opengl"
)

var (
	// isMetalSupported represents whether Metal is supported.
	isMetalSupported = true

	// isMetalSupportedOnce initializes isMetalSupported.
	//
	// Use sync.Once instead of init function to avoid init-order dependency (#886).
	isMetalSupportedOnce sync.Once
)

func Get() driver.Graphics {
	isMetalSupportedOnce.Do(func() {
		// On old mac devices like iMac 2011, Metal is not supported (#779).
		if _, err := mtl.CreateSystemDefaultDevice(); err != nil {
			isMetalSupported = false
			return
		}
		// On macOS 10.11 El Capitan, there is a rendering issue on Metal (#781).
		// Use the OpenGL in macOS 10.11 or older.
		if C.getMacOSMinorVersion() <= 11 {
			isMetalSupported = false
			return
		}
	})
	if isMetalSupported {
		return metal.Get()
	}
	return opengl.Get()
}
