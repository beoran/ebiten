package primitives

import "github.com/chewxy/math32"

const Epsilon = 0.001

//
// Normalizes vector.
//
func (vector *Vertex) Normalize() float32 {
   invLength := float32(1.0);

   length     := math32.Hypot(vector[0], vector[1])
   if length > 0.0 { 
		invLength  = 1.0 / length
   } 

   vector[0] *= invLength;
   vector[1] *= invLength;

   return length;
}

//
//  Tests on which side of the line point is placed.
//  Positive value will be returned if point is on half plane
//  determined by normal vector. Negative value will be returned
//  if point is on negative half plane determined by normal vector.
//  Zero will be returned if point lie on the line.
func (point Vertex) TestLineSide(origin Vertex, normal Vertex) int {
   c := -(origin[0] * normal[0] + origin[1] * normal[1]);
   d :=   point[0]  * normal[0] + point[1]  * normal[1] + c;
   if d < 0.0 {
      return -1
   } else if d > 0.0 {
      return 1
   } else  {
      return 0
   }
}


//  Tests if point is inside of the triangle defined by vertices v0, v1 and v2.
//  Order of vertices does not matter
func (point Vertex) IsPointInTriangle(v0, v1, v2 Vertex) bool {
   edgeNormal0 := Vertex{ -(v1[1] - v0[1]), v1[0] - v0[0] }
   edgeNormal1 := Vertex{ -(v2[1] - v1[1]), v2[0] - v1[0] }
   edgeNormal2 := Vertex{ -(v0[1] - v2[1]), v0[0] - v2[0] }

   edgeSide0 := point.TestLineSide(v0, edgeNormal0)
   edgeSide1 := point.TestLineSide(v1, edgeNormal1)
   edgeSide2 := point.TestLineSide(v2, edgeNormal2)

   if (edgeSide1 != 0 && edgeSide1 != 0 && edgeSide2 != 0) {
      return (edgeSide0 == edgeSide1) && (edgeSide0 == edgeSide2);
   } else if (0 == edgeSide0) {
      return (edgeSide1 == edgeSide2);
   } else if (0 == edgeSide1) {
      return (edgeSide0 == edgeSide2);
   } else /*if (0 == edge_side_2)*/ {
      return (edgeSide0 == edgeSide1);
   }
}


//
//  Tests for intersection of lines defined by points { v0, v1 }
//  and { p0, p1 }.
//
//  Returns true if intersection point was determined. If pointers
//  are provided time and exact point of intersection will be returned.
//  If test fails false will be returned. Intersection point and time
//  variables will not be altered in this case.
//
//  Intersection time is in { v0, v1 } line space.
//
func primIntersectSegment(v0, v1, p0, p1 Vertex) (found bool, point, t Vertex) {
   var num, denom, time float32;

   denom = (p1[1] - p0[1]) * (v1[0] - v0[0]) - (p1[0] - p0[0]) * (v1[1] - v0[1]);

   if (math32.Abs(denom) == 0.0) {
      return false, point, t;
   }
   
   num = (p1[0] - p0[0]) * (v0[1] - p0[1]) - (p1[1] - p0[1]) * (v0[0] - p0[0]);

   time = (num / denom);
   t[0] = time;
   num2 := (v1[0] - v0[0]) * (v0[1] - p0[1]) - (v1[1] - v0[1]) * (v0[0] - p0[0]);
   t[1] = (num2 / denom);

   point[0] = v0[0] + time * (v1[0] - v0[0]);
   point[1] = v0[1] + time * (v1[1] - v0[1]);

   return true, point, t;
}


//  Compares two points for equality.
//  This is not an exact comparison but it is sufficient for our needs.
func (pointA Vertex) Equal(pointB Vertex) bool {
   return (math32.Abs(pointA[0] - pointB[0]) < Epsilon) && 
		   (math32.Abs(pointA[1] - pointB[1]) < Epsilon);
}

