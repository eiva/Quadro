#pragma once
#include <math.h>
/*
 * For model position:
 * X: front;
 * Y: Right;
 * Z: Up;
 * For model Rotation:
 * Clockwise rotation used.
 */
class Vector3{
public:
   float X, Y, Z;

   Vector3(int32_t x, int32_t y, int32_t z) :X(x), Y(y), Z(z){}
   Vector3(float x, float y, float z) :X(x), Y(y), Z(z){}
   Vector3() :X(0.0f), Y(0.0f), Z(0.0f){}
   Vector3(const Vector3& cp) :X(cp.X), Y(cp.Y), Z(cp.Z){}

   void Set(const float &xValue, const float &yValue, const float &zValue){
      X = xValue;
      Y = yValue;
      Z = zValue;
   }

   // Method to normalise a vector
   void Normalise(){
      // Calculate the magnitude of our vector
      float magnitude = sqrt((X * X) + (Y * Y) + (Z * Z));

      // As long as the magnitude isn't zero, divide each element by the magnitude
      // to get the normalised value between -1 and +1
      if (magnitude != 0){
         X /= magnitude;
         Y /= magnitude;
         Z /= magnitude;
      }
   }

   // Static method to calculate and return the scalar dot product of two vectors
   //
   // Note: The dot product of two vectors tell us things about the angle between
   // the vectors. That is, it tells us if they are pointing in the same direction
   // (i.e. are they parallel? If so, the dot product will be 1), or if they're
   // perpendicular (i.e. at 90 degrees to each other) the dot product will be 0,
   // or if they're pointing in opposite directions then the dot product will be -1.
   //
   // Usage example: double foo = Vec3<double>::dotProduct(vectorA, vectorB);
   static float DotProduct(const Vector3 &vec1, const Vector3 &vec2) {
      return vec1.X * vec2.X + vec1.Y * vec2.Y + vec1.Z * vec2.Z;
   }

   // Non-static method to calculate and return the scalar dot product of this vector and another vector
   //
   // Usage example: double foo = vectorA.dotProduct(vectorB);
   float dotProduct(const Vector3 &vec) const {
      return X * vec.X + Y * vec.Y + Z * vec.Z;
   }

   // Static method to calculate and return a vector which is the cross product of two vectors
   //
   // Note: The cross product is simply a vector which is perpendicular to the plane formed by
   // the first two vectors. Think of a desk like the one your laptop or keyboard is sitting on.
   // If you put one pencil pointing directly away from you, and then another pencil pointing to the
   // right so they form a "L" shape, the vector perpendicular to the plane made by these two pencils
   // points directly upwards.
   //
   // Whether the vector is perpendicularly pointing "up" or "down" depends on the "handedness" of the
   // coordinate system that you're using.
   //
   // Further reading: http://en.wikipedia.org/wiki/Cross_product
   //
   // Usage example: Vec3<double> crossVect = Vec3<double>::crossProduct(vectorA, vectorB);
   static Vector3 crossProduct(const Vector3 &vec1, const Vector3 &vec2) {
      return Vector3(vec1.Y * vec2.Z - vec1.Z * vec2.Y, vec1.Z * vec2.X - vec1.X * vec2.Z, vec1.X * vec2.Y - vec1.Y * vec2.X);
   }

   // Easy adders
   void AddX(const float value) { X += value; }
   void AddY(const float value) { Y += value; }
   void AddZ(const float value) { Z += value; }

   static float getDistance(const Vector3 &v1, const Vector3 &v2) {
      const float dx = v2.X - v1.X;
      const float dy = v2.Y - v1.Y;
      const float dz = v2.Z - v1.Z;

      return sqrt(dx * dx + dy * dy + dz * dz);
   }
   // ------------ Overloaded operators ------------

   // Overloaded addition operator to add Vec3s together
   Vector3 operator+(const Vector3 &vector) const {
      return Vector3(X + vector.X, Y + vector.Y, Z + vector.Z);
   }

   // Overloaded add and asssign operator to add Vec3s together
   void operator+=(const Vector3 &vector) {
      X += vector.X;
      Y += vector.Y;
      Z += vector.Z;
   }

   // Overloaded subtraction operator to subtract a Vector3 from another Vec3
   Vector3 operator-(const Vector3 &vector) const {
      return Vector3(X - vector.X, Y - vector.Y, Z - vector.Z);
   }

   // Overloaded subtract and asssign operator to subtract a Vector3 from another Vec3
   void operator-=(const Vector3 &vector) {
      X -= vector.X;
      Y -= vector.Y;
      Z -= vector.Z;
   }

   // Overloaded multiplication operator to multiply two Vec3s together
   Vector3 operator*(const Vector3 &vector) const {
      return Vector3(X * vector.X, Y * vector.Y, Z * vector.Z);
   }

   // Overloaded multiply operator to multiply a vector by a scalar
   Vector3 operator*(const float &value) const {
      return Vector3(X * value, Y * value, Z * value);
   }

   // Overloaded multiply and assign operator to multiply a vector by a scalar
   void operator*=(const float &value) {
      X *= value;
      Y *= value;
      Z *= value;
   }

   // Overloaded multiply operator to multiply a vector by a scalar
   Vector3 operator/(const float &value) const {
      return Vector3(X / value, Y / value, Z / value);
   }

   // Overloaded multiply and assign operator to multiply a vector by a scalar
   void operator/=(const float &value){
      X /= value;
      Y /= value;
      Z /= value;
   }
};
