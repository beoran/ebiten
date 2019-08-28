// Polygon triangulation with holes. Based On Allegro5's triangulator.c by
//* by Michał Cichoń, under the Allegro5 license.
 //
package primitives

import "math"


const POLY_VERTEX_ATTR_REFLEX 	= 0x0001
const POLY_VERTEX_ATTR_EAR_CLIP = 0x0002
const POLY_VERTEX_ATTR_ALL      =  (POLY_VERTEX_ATTR_REFLEX | POLY_VERTEX_ATTR_EAR_CLIP)

type PolyEmitTriangle func(int, int, int, interface{})

type Poly struct {
   VertexBuffer []float32
   VertexStride uint
   VertexCount uint
   SplitIndices []int;
   SplitStride uint
   SplitCount uint
   emit PolyEmitTriangle
   UserData interface{}
   // Vertices, in pairwise format. 
   // The first vertex x is Vertices[0], the first vertex x is Vertices[1], 
   // The second vertex x is Vertices[2], the second vertex y is Vertices[3], 
   // and so forth.
   Vertices []float32
   Reflexes []uint
   Ears []uint
}

type PolySplit struct {
	Begin int
	Size uint
	Point Vertex
	MaxIndex int
}


type Vertex [2]float32

//
//  Seek for closest intersection point of polygon with a ray directed
//  to the right.
//
//  Returns true if intersection was found. 'point', 'edge0' and 'edge1'
//  are set to proper values (if not NULL).
//  Returns false if intersection was not fount and none of above
//  are modified.
 //
func polyFindClosestIntersection(vertices[]float32, vertex Vertex) (found bool, point, edge0, edge1 Vertex) {
   var v0, v1 Vertex
   var bestPoint Vertex
   var bestE0 Vertex;
   var bestE1 Vertex;

   // Assemble line pointing to the right. //
   v0    = vertex
   v1[0] = v0[0] + 1.0
   v1[1] = v0[1]

   // Seek for the closest intersection. //
   bestT := float32(math.MaxFloat32);
   for i := 0 ; i < len(vertices) ; i+=2  {

      p0 := Vertex{vertices[i], vertices[i+i]};
      j :=  i + 2
      if j > len(vertices) { 
			j = 0
	  }
      p1 := Vertex{vertices[j], vertices[j+1]};

      // Perform quick rejection.
      //
      // We are interested only with outer edges. That mean only those
      // whose go from the bottom to the up. Note we cannot intersect
      // parallel line.
      //
      // Second condition reject edges lying completely on the left
      // side of ray origin.
      //
      if (p0[1] < p1[1]) || (p0[0] <= vertex[0] && p1[0] <= vertex[0]) {
         continue;
      }
         
      found, intersection, t := primIntersectSegment(v0, v1, p0, p1)

      if found && (t[1] >= 0.0) && (t[1] <= 1.0) && (t[0] >= 0.0) && (t[0] < bestT) {

         bestT        = t[0];
         bestPoint[0] = intersection[0];
         bestPoint[1] = intersection[1];
         bestE0       = p0;
         bestE1       = p1;
      }
   }

   found = false 	

   if (bestE0 >= 0) {
	   found = true
   }
   
   return found, bestPoint, bestE0, bestE1
}


//
//  Seek for the best vertex in polygon for doing split.
//
//  Returns vertex after which split (hole) vertices
//  can be inserted.
//
func polyFindOuterSplitVertex(polygon Poly, split PolySplit) (bool, Vertex) {
    var p Vertex
	found, intersection, edge0, edge1 := polyFindClosestIntersection(polygon.Vertices, split.Point)
	if !found { 
		return false, intersection
	}
	
	// Maybe we hit an edge vertex? 
	if split.Point.Equal(edge0) {
		return true, edge0
	}
		
	if split.Point.Equal(edge1) {
		return true, edge1
	}
	
	// No luck. Pick point which most far away from us. 
	if (edge0[0] > edge1[0]) {
		p = edge0;
	} else {
		p = edge1;	
    }
	
	// Seek in reflex vertices.
	var bestVertex Vertex
	bestDistance := float32(math.MaxFloat32)
	
	for i:= 0; i < len(polygon.Reflexes) ; i+=2 { 
		reflex := Vertex{polygon.Reflexes[i]

   for (reflex_item = _al_list_front(polygon->reflex_list); reflex_item; reflex_item = _al_list_next(polygon->reflex_list, reflex_item)) {

      _AL_LIST_ITEM* reflex_vertex = (_AL_LIST_ITEM*)_al_list_item_data(reflex_item);

      float* reflex_point = (float*)_al_list_item_data(reflex_vertex);

      if (_al_prim_is_point_in_triangle(reflex_point, split->point, p, intersection)) {

         float diff_x = reflex_point[0] - split->point[0];
         float diff_y = reflex_point[1] - split->point[1];

         float dist = diff_x * diff_x + diff_y * diff_y;

         if (dist < best_distance) {

            best_distance = dist;
            best_vertex   = reflex_vertex;
         }
      }
   }

   if (NULL != best_vertex)
      vertex = best_vertex;

//# if POLY_DEBUG
//   p0 = (float*)_al_list_item_data(vertex);
//   al_draw_line(p0[0], p0[1], intersection[0], intersection[1], al_map_rgb(255, 0, 0), 0.0f);
//# endif

   return vertex;
}


# define POLY_VERTEX(index)      ((float*)(((uint8_t*)polygon->vertex_buffer) + (index) * polygon->vertex_stride))
# define POLY_SPLIT(index)       (*((int*)(((uint8_t*)polygon->SplitIndices) + (index) * polygon->SplitStride)))
# define POLY_SPLIT_INDEX(split) (((uint8_t*)split - (uint8_t*)polygon->SplitIndices) / polygon->SplitStride)


//
//  Create list of the splits.
 //
static _AL_LIST* poly_create_split_list(POLY* polygon)
{
   int i;
   int last_split;

   _AL_LIST*       list   = _al_list_create_static(polygon->SplitCount);
   POLY_SPLIT* splits = (POLY_SPLIT*)al_malloc(polygon->SplitCount * sizeof(POLY_SPLIT));

   if ((NULL == list) || (NULL == splits)) {

      _al_list_destroy(list);
      al_free(splits);

      return NULL;
   }

   // Set list destructor, so we will don't worry about
    * manually releasing allocated memory.
    //
   _al_list_set_dtor(list, poly_split_list_dtor);
   _al_list_set_user_data(list, splits);

   last_split = POLY_SPLIT(0);
   for (i = 1; i < (int)polygon->SplitCount; ++i) {

      int j;
      float max;
      POLY_SPLIT* split = splits + i;
      _AL_LIST_ITEM* where;

      split->begin = last_split;
      split->size  = POLY_SPLIT(i) - last_split;
      last_split   = last_split + split->size;

      max = -FLT_MAX;
      split->max_index = -1;
      for (j = split->begin; j < split->begin + (int)split->size; ++j) {

         float* point = POLY_VERTEX(j);

         if (point[0] >= max) {

            max               = point[0];
            split->point      = point;
            split->max_index  = j;
         }
      }

      split->max_index -= split->begin;

      ASSERT(split->max_index >= 0);

      where = NULL;
      if (!_al_list_is_empty(list)) {

         for (where = _al_list_front(list); where; where = _al_list_next(list, where)) {

            POLY_SPLIT* local_split = (POLY_SPLIT*)_al_list_item_data(where);

            if (local_split->point[0] < max)
               break;
         }
      }

      if (NULL == where)
         _al_list_push_back(list, split);
      else
         _al_list_insert_before(list, where, split);
   }

   return list;
}


//
//  Release memory allocated by split list. This method
//  serve as callback to the list.
 //
static void poly_split_list_dtor(void* user_data)
{
   // This is array of POLY_SPLIT. //
   al_free(user_data);
}


//
//  Perform initialization step to polygon triangulation.
//
//  Three linked list are initialized: vertex_list, reflex_list
//  and ear_list.
//
//  All provided splits (holes) are resolved in this step.
//  Therefore at the end we have simple polygon.
 //
static bool poly_initialize(POLY* polygon)
{
   _AL_LIST* vertex_list;
   _AL_LIST* reflex_list;
   _AL_LIST* ear_list;
   _AL_LIST* split_list;
   _AL_LIST_ITEM* split_item;
   size_t vertex_count;
   bool use_split_list;
   size_t j;
   int i;

# if POLY_DEBUG
   if (g_poly_debug_split_step >= 0)
      polygon->SplitCount = min(g_poly_debug_split_step, polygon->SplitCount);
# endif

   // Every hole add two extra vertices to the list. //
   vertex_count = polygon->vertex_count + (polygon->SplitCount - 1) * 2;

   // Create lists for polygon. //
   vertex_list = _al_list_create_static(vertex_count);
   reflex_list = _al_list_create_static(vertex_count);
   ear_list    = _al_list_create_static(vertex_count);

   if (polygon->SplitCount > 1) {

      split_list     = poly_create_split_list(polygon);
      use_split_list = true;
   }
   else {
      split_list = NULL;
      use_split_list = false;
   }

   if ((NULL == vertex_list) || (NULL == reflex_list) || (NULL == ear_list) || (use_split_list && (NULL == split_list))) {

      _al_list_destroy(vertex_list);
      _al_list_destroy(reflex_list);
      _al_list_destroy(ear_list);
      _al_list_destroy(split_list);

      return false;
   }

   // Store lists in polygon. //
   polygon->vertex_list = vertex_list;
   polygon->reflex_list = reflex_list;
   polygon->ear_list    = ear_list;

   // Push main polygon outline. //
   for (i = 0; i < POLY_SPLIT(0); ++i)
      _al_list_push_back(vertex_list, POLY_VERTEX(i));

   if (use_split_list) {

# if POLY_DEBUG
       int current_split = 0;
# endif

       poly_classify_vertices(vertex_list, reflex_list, NULL);

      // Resolve all holes. //
      for (split_item = _al_list_front(split_list); split_item; split_item = _al_list_next(split_list, split_item)) {

         _AL_LIST_ITEM* first_vertex;
         _AL_LIST_ITEM* last_vertex;
         POLY_SPLIT* split;

# if POLY_DEBUG
         if (g_poly_debug_split_step >= 0 && current_split >= g_poly_debug_split_step)
             break;

         ++current_split;
# endif

         split = (POLY_SPLIT*)_al_list_item_data(split_item);

         first_vertex = poly_find_outter_split_vertex(polygon, split);

         if (NULL == first_vertex)
            break;

         // Insert hole vertices. //
         last_vertex = first_vertex;
         for (j = 0; j <= split->size; ++j)
            last_vertex = _al_list_insert_after(vertex_list, last_vertex,
               POLY_VERTEX(split->begin + ((j + split->max_index) % split->size)));
         last_vertex = _al_list_insert_after(vertex_list, last_vertex, _al_list_item_data(first_vertex));

         _al_list_remove(reflex_list, first_vertex);

         poly_classify_vertices_in_range(first_vertex, _al_list_next(vertex_list, last_vertex), vertex_list, reflex_list, NULL);
      }

      _al_list_destroy(split_list);

      poly_classify_vertices(vertex_list, NULL, ear_list);
   }
   else
      // Initialize reflex and ear vertex lists. //
      poly_classify_vertices(vertex_list, reflex_list, ear_list);

   return true;
}

# undef POLY_VERTEX
# undef POLY_SPLIT
# undef POLY_SPLIT_INDEX


//
//  Compute polygon vertex attributes. Currently supported attributes are:
//    - ear clip
//    - reflex
//
//  Vertex is an ear clip when triangle made by their edges does not contain
//  any other vertices.
//
//  Vertex is a reflex when angle between edges is greater or equal
//  to 180 degrees.
//
//  Optionally function may take reflex list (if not NULL). In this case test
//  for ear clip is performed only on reflex vertices not on entire polygon.
//  This is used for triangulation speed optimization, because convex
//  (non reflect) vertices will never became reflex.
//
//  Flags are used to determine for which attribute we want to check.
//  Use POLY_VERTEX_ATTR_ALL to test for all attributes.
 //
static int poly_compute_vertex_attributes(_AL_LIST* vertices, _AL_LIST_ITEM* item, int flags, _AL_LIST* reflex)
{
   size_t i;
   _AL_LIST_ITEM* prev = _al_list_previous_circular(vertices, item);
   _AL_LIST_ITEM* next = _al_list_next_circular(vertices, item);
   _AL_LIST_ITEM* point = NULL;
   int result = 0;
   float* v0;
   float* v1;
   float* v2;
   float cross;

   // Oops, degenerate triangle in store.
   if (_al_list_size(vertices) < 3)
      return result;

   // Get points.
   v0 = (float*)_al_list_item_data(prev);
   v1 = (float*)_al_list_item_data(item);
   v2 = (float*)_al_list_item_data(next);

   // Compute the cross product between the two edges
   cross = (v0[0] - v1[0]) * (v2[1] - v1[1]) - (v0[1] - v1[1]) * (v2[0] - v1[0]);

# if POLY_DEBUG
   if (g_poly_debug_step == g_poly_debug_step_current) {

      float step = 0.25f * 0.0f;

      float dir0[2] = { v0[0] - v1[0], v0[1] - v1[1] };
      float dir2[2] = { v2[0] - v1[0], v2[1] - v1[1] };

      POLY_DEBUG_TEXT(v1[0], v1[1], "%d", cross > 0);

      al_draw_line(v1[0], v1[1], v1[0] + dir0[0] * step, v1[1] + dir0[1] * step, al_map_rgb(255, 0, 0), 4.0f * g_poly_debug_scale);
      al_draw_line(v1[0], v1[1], v1[0] + dir2[0] * step, v1[1] + dir2[1] * step, al_map_rgb(255, 0, 0), 4.0f * g_poly_debug_scale);
   }
# endif

   // If internal angle is less than 180 degrees then we may
   // have an ear clip vertex, otherwise we have reflex.
   if (cross >= 0)
   {
      if (flags & POLY_VERTEX_ATTR_EAR_CLIP)
      {
         _AL_LIST_ITEM* start;
         _AL_LIST* search_list;
         size_t size;

         if (reflex) {

            search_list = reflex;
            size        = _al_list_size(search_list);
            start       = _al_list_front(reflex);
         }
         else {
            search_list = vertices;
            size        = _al_list_size(search_list) - 3;
            start       = _al_list_next_circular(search_list, next);
         }

         // Check for ear vertex.
         point = start;
         for (i = 0; i < size; ++i, point = _al_list_next_circular(search_list, point)) {

            float* v;

            // TODO: optimize this by removing if
            if (search_list == reflex)
               v = (float*)_al_list_item_data((_AL_LIST_ITEM*)_al_list_item_data(point));
            else
               v = (float*)_al_list_item_data(point);

            // Ignore vertices which belong to the triangle.
            if ((v == v0) || (v == v1) || (v == v2))
               continue;

            // Set point to NULL if we find any point inside the triangle.
            if (_al_prim_is_point_in_triangle(v, v0, v1, v2)) {
               point = NULL;
               break;
            }
         }

         // If point is not NULL, we have ear vertex.
         if ((NULL != point) || _al_list_is_empty(search_list))
            result |= POLY_VERTEX_ATTR_EAR_CLIP;
      }
   }
   else if (flags & POLY_VERTEX_ATTR_REFLEX)
      result |= POLY_VERTEX_ATTR_REFLEX;

   return result;
}


//
//  Classify all vertices into reflex or ear group.
//
//  One of target group may be NULL so it will be simply ignored.
 //
static void poly_classify_vertices(_AL_LIST* vertices, _AL_LIST* reflex, _AL_LIST* ear)
{
   poly_classify_vertices_in_range(_al_list_front(vertices), NULL, vertices, reflex, ear);
}


//
//  Classify selected range of vertices [begin, end) into
//  reflex or ear group.
//
//  One of target group may be NULL so it will be simply ignored.
 //
static void poly_classify_vertices_in_range(_AL_LIST_ITEM* begin, _AL_LIST_ITEM* end, _AL_LIST* vertices, _AL_LIST* reflex, _AL_LIST* ear)
{
   _AL_LIST_ITEM* item = NULL;
   int attribute_mask = 0;

   if (NULL != ear)
      attribute_mask |= POLY_VERTEX_ATTR_EAR_CLIP;

   if (NULL != reflex)
      attribute_mask |= POLY_VERTEX_ATTR_REFLEX;

   for (item = begin; item != end; item = _al_list_next(vertices, item))
   {
      int attr = poly_compute_vertex_attributes(vertices, item, attribute_mask, NULL);

      if (0 == attr)
         continue;

      if (attr & POLY_VERTEX_ATTR_EAR_CLIP)
         _al_list_push_back(ear, item);

      if (attr & POLY_VERTEX_ATTR_REFLEX)
         _al_list_push_back(reflex, item);
   }
}


//
//  Reclassify vertex. After triangle was emitted one vertex
//  is removed from the list. Two neighbor vertices may
//  change their attributes. In this place we have general
//  function which update lists to match new attributes
//  of provided vertex.
 //
static void poly_update_vertex_attributes(_AL_LIST* vertices, _AL_LIST* reflex, _AL_LIST* ear, _AL_LIST_ITEM* vertex)
{
   _AL_LIST_ITEM* item;

   int attr = poly_compute_vertex_attributes(vertices, vertex, POLY_VERTEX_ATTR_ALL, reflex);

   // Update reflex list only if an attribute change.
   item = _al_list_find_first(reflex, vertex);
   if (attr & POLY_VERTEX_ATTR_REFLEX) {

      if (NULL == item)
         _al_list_push_back(reflex, vertex);

      _al_list_remove(ear, vertex);
   }
   else {

      if (NULL != item)
         _al_list_erase(reflex, item);

      item = _al_list_find_first(ear, vertex);
      if (attr & POLY_VERTEX_ATTR_EAR_CLIP) {

         if (NULL == item)
            _al_list_push_front(ear, vertex);
      }
      else {

         if (NULL != item)
            _al_list_erase(ear, item);
      }
   }
}


//
//  Triangulator iterate trough list of ear vertices
//  and clip isolated triangles. This process repeats
//  until there are ear vertices.
 //
static void poly_do_triangulate(POLY* polygon)
{
# define VERTEX_INDEX(vertex) ((((uint8_t*)vertex) - ((uint8_t*)polygon->vertex_buffer)) / polygon->vertex_stride)

# if POLY_DEBUG
   g_poly_debug_step_current = 0;

   {
      _AL_LIST_ITEM* item;
      _AL_LIST_ITEM* next;
      int* histogram = al_calloc(_al_list_size(polygon->vertex_list), sizeof(int));

      //std::map<float*, int> histogram;//

      int index = 0;
      for (item = _al_list_front(polygon->vertex_list); item; item = _al_list_next(polygon->vertex_list, item)) {

         float* point0;
         float* point1;
         float n[2];
         float l, r;
         char status[3] = { 0 };
         int status_index = 0;

         next = _al_list_next_circular(polygon->vertex_list, item);

         point0 = (float*)_al_list_item_data(item);
         point1 = (float*)_al_list_item_data(next);

         n[0] = -(point1[1] - point0[1]);
         n[1] =   point1[0] - point0[0];

         l = 2.0f * sqrtf(n[0] * n[0] + n[1] * n[1]);

         n[0] /= l;
         n[1] /= l;

         r = 1.0f * 0.0f;

         al_draw_line(point0[0] + n[0] * r, point0[1] + n[1] * r, point1[0] + n[0] * r, point1[1] + n[1] * r, al_map_rgba(255, 0, 255, 128), r * g_poly_debug_scale);

         if (_al_list_contains(polygon->reflex_list, item))
            status[status_index++] = 'R';
         if (_al_list_contains(polygon->ear_list, item))
            status[status_index++] = 'E';

         POLY_DEBUG_TEXT_LINE(point0[0], point0[1], -histogram[VERTEX_INDEX(point0)], "%s %d", status, index++);

         ++histogram[VERTEX_INDEX(point0)];
      }

      al_free(histogram);
   }
# endif

   // Repeat until there are ear clips.
   while (!_al_list_is_empty(polygon->ear_list)) {

      _AL_LIST_ITEM* ear_item;
      _AL_LIST_ITEM* vertex_item;
      _AL_LIST_ITEM* next;
      _AL_LIST_ITEM* prev;
      float* v0;
      float* v1;
      float* v2;

      // Get point and corresponding item on polygon->vertex_list list.
      ear_item    = _al_list_front(polygon->ear_list);
      vertex_item = (_AL_LIST_ITEM*)_al_list_item_data(ear_item);
      prev        = _al_list_previous_circular(polygon->vertex_list, vertex_item);
      next        = _al_list_next_circular(polygon->vertex_list,     vertex_item);
      v0 = (float*)_al_list_item_data(prev);
      v1 = (float*)_al_list_item_data(vertex_item);
      v2 = (float*)_al_list_item_data(next);

# if POLY_DEBUG
      if (g_poly_debug_step == g_poly_debug_step_current) {
         _AL_LIST_ITEM* item;
         ALLEGRO_COLOR color;
         int second = 0;

         al_draw_filled_triangle(v0[0], v0[1], v1[0], v1[1], v2[0], v2[1], al_map_rgba(0, 0, 255, 64));

         for (item = _al_list_front(polygon->vertex_list); item; item = _al_list_next(polygon->vertex_list, item)) {

            float* point = (float*)_al_list_item_data(item);
            al_draw_filled_circle(point[0], point[1], 6.0f * g_poly_debug_scale, al_map_rgb(255, 255, 255));
         }

         for (item = _al_list_front(polygon->reflex_list); item; item = _al_list_next(polygon->reflex_list, item)) {

            float* point = (float*)_al_list_item_data((_AL_LIST_ITEM*)_al_list_item_data(item));

            al_draw_filled_circle(point[0], point[1], 6.0f * g_poly_debug_scale, al_map_rgb(255, 255, 0));
         }

         color = al_map_rgb(255, 0, 0);
         second = _al_list_size(polygon->ear_list) - 1;
         for (item = _al_list_back(polygon->ear_list); item; item = _al_list_previous(polygon->ear_list, item), --second) {

            float* point = (float*)_al_list_item_data((_AL_LIST_ITEM*)_al_list_item_data(item));

            if (second == 0)
               color = al_map_rgb(0, 255, 0);
            else if (second == 1)
               color = al_map_rgb(0, 0, 255);

            al_draw_circle(point[0], point[1], 9.0f * g_poly_debug_scale, color, 2.0f * g_poly_debug_scale);
         }
      }
      if (g_poly_debug_step >= 0 && g_poly_debug_step_current >= g_poly_debug_step)
         break;
      g_poly_debug_step_current++;
# endif

      // Emit triangle.
      polygon->emit(VERTEX_INDEX(v0), VERTEX_INDEX(v1), VERTEX_INDEX(v2), polygon->userdata);

      // Remove polygon->ear_list clip from all lists.
      _al_list_erase(polygon->ear_list,      ear_item);
      _al_list_erase(polygon->vertex_list, vertex_item);
      _al_list_erase(polygon->reflex_list, _al_list_find_first(polygon->reflex_list, vertex_item));

      // Update attributes of corner vertices.
      poly_update_vertex_attributes(polygon->vertex_list, polygon->reflex_list, polygon->ear_list, prev);
      poly_update_vertex_attributes(polygon->vertex_list, polygon->reflex_list, polygon->ear_list, next);
   }

# undef VERTEX_INDEX
}


// Function: al_triangulate_polygon
//  General triangulation function.
 //
bool al_triangulate_polygon(
   const float* vertices, size_t vertex_stride, const int* vertex_counts,
   void (*emit_triangle)(int, int, int, void*), void* userdata)
{
   POLY polygon;
   int vertex_count;
   int SplitCount;
   int *splits;
   int i;
   bool ret;

   for (i = 0; vertex_counts[i] > 0; i++) {
      // do nothing //
   }
   ASSERT(i > 0);
   SplitCount = i;

   splits = malloc(SplitCount * sizeof(int));
   if (!splits) {
      return false;
   }
   vertex_count = 0;
   for (i = 0; i < SplitCount; i++) {
      vertex_count += vertex_counts[i];
      splits[i] = vertex_count;
   }

   memset(&polygon, 0, sizeof(polygon));
   polygon.vertex_buffer = vertices;
   polygon.vertex_stride = vertex_stride;
   polygon.vertex_count  = vertex_count;
   polygon.SplitIndices = splits;
   polygon.SplitStride  = sizeof(int);   // XXX can simplify code now //
   polygon.SplitCount   = SplitCount;
   polygon.emit          = emit_triangle;
   polygon.userdata      = userdata;

   if (poly_initialize(&polygon)) {

      poly_do_triangulate(&polygon);

      _al_list_destroy(polygon.vertex_list);
      _al_list_destroy(polygon.reflex_list);
      _al_list_destroy(polygon.ear_list);

      ret = true;
   }
   else {
      ret = false;
   }

   free(splits);

   return ret;
}
*/
// vim: set sts=3 sw=3 et: //
