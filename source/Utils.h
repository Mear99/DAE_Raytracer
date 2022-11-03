#pragma once
#include <cassert>
#include <fstream>
#include "Math.h"
#include "DataTypes.h"

//#define SPHERE_ANALYTIC
#define SPHERE_GEOMETRIC

//#define TRIANGLE_NAIVE
#define TRIANGLE_MT

namespace dae
{
	namespace GeometryUtils
	{
#pragma region Sphere HitTest
		//SPHERE HIT-TESTS
		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
#if defined(SPHERE_ANALYTIC)

			// Analytic solution
			Vector3 L{ray.origin - sphere.origin};
			float a = ray.direction.SqrMagnitude();
			float b = Vector3::Dot(2*ray.direction,L);
			float c = L.SqrMagnitude() - (sphere.radius*sphere.radius);

			float discriminant{ b * b - 4 * a * c };
			if (discriminant > 0) {

				float t = (-b - sqrt(discriminant)) / (2 * a);
				if (t < ray.min || t > ray.max) {
					t = (-b + sqrt(discriminant)) / (2 * a);
					if (t < ray.min || t > ray.max) {
						return false;
					}
				}

				if (ignoreHitRecord) {
					return true;
				}

				hitRecord.didHit = true;
				hitRecord.materialIndex = sphere.materialIndex;
				hitRecord.t = t;
				hitRecord.origin = ray.origin + ray.direction * t;
				hitRecord.normal = hitRecord.origin - sphere.origin;

				return true;
			}
			return false;


#elif defined(SPHERE_GEOMETRIC)

			// Geometric Solution
			// hypothenuse
			Vector3 L{ sphere.origin - ray.origin };
			// adjacent side
			float tca{ Vector3::Dot(L, ray.direction) };
			// opposite side squared
			float odSqrd{ L.SqrMagnitude() - tca * tca };

			if (odSqrd <= sphere.radius * sphere.radius) {

				float thc{ sqrtf(sphere.radius * sphere.radius - odSqrd) };
				float t{ tca - thc };
				if (t < ray.min || t > ray.max) {
					t = tca + thc;
					if (t < ray.min || t > ray.max) {
						return false;
					}
				}

				if (ignoreHitRecord) {
					return true;
				}

				hitRecord.didHit = true;
				hitRecord.materialIndex = sphere.materialIndex;
				hitRecord.t = t;
				hitRecord.origin = ray.origin + ray.direction * t;
				hitRecord.normal = (hitRecord.origin - sphere.origin).Normalized();

				return true;
			}
			return false;
#endif
		}

		inline bool HitTest_Sphere(const Sphere& sphere, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Sphere(sphere, ray, temp, true);
		}
#pragma endregion
#pragma region Plane HitTest
		//PLANE HIT-TESTS
		inline bool HitTest_Plane(const Plane& plane, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			float t = Vector3::Dot(plane.origin - ray.origin, plane.normal) / Vector3::Dot(ray.direction, plane.normal);

			if (t >= ray.min && t <= ray.max) {
				if (ignoreHitRecord) {
					return true;
				}

				hitRecord.didHit = true;
				hitRecord.materialIndex = plane.materialIndex;
				hitRecord.t = t;
				hitRecord.origin = ray.origin + ray.direction * t;
				hitRecord.normal = plane.normal.Normalized();

				return true;
			}

			return false;
		}

		inline bool HitTest_Plane(const Plane& plane, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Plane(plane, ray, temp, true);
		}
#pragma endregion
#pragma region Triangle HitTest
		//TRIANGLE HIT-TESTS
		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			// Culling checks
			float dotProduct{ Vector3::Dot(triangle.normal, ray.direction) };
			// Ray doesn't hit the plane
			if (	dotProduct == 0 
				||	(dotProduct > 0 && triangle.cullMode == TriangleCullMode::BackFaceCulling && !ignoreHitRecord)	// backface not shadow ray
				||	(dotProduct < 0 && triangle.cullMode == TriangleCullMode::FrontFaceCulling && !ignoreHitRecord)	// frontface not shadow ray
				||	(dotProduct > 0 && triangle.cullMode == TriangleCullMode::FrontFaceCulling && ignoreHitRecord)	// frontface shadow ray
				||	(dotProduct < 0 && triangle.cullMode == TriangleCullMode::BackFaceCulling && ignoreHitRecord)	// backface shadow ray	
			) {
				return false;
			}

#if defined(TRIANGLE_NAIVE)
			// Naive Triangle Ray Intersection
			// Plane hit check
			Vector3 triangleCenter{ (triangle.v0 + triangle.v1 + triangle.v2) / 3 };
			float t = Vector3::Dot(triangleCenter - ray.origin, triangle.normal) / Vector3::Dot(ray.direction, triangle.normal);

			if (t < ray.min || t > ray.max) {
				return false;
			}

			// Triangle check
			Vector3 hitPoint{ ray.origin + ray.direction * t };

			//Side A
			Vector3 side{ triangle.v1 - triangle.v0 };
			Vector3 pointToSide{ hitPoint - triangle.v0 };
			if (Vector3::Dot(triangle.normal, Vector3::Cross(side, pointToSide)) < 0) {
				return false;
			}
				
			//Side B
			side = { triangle.v2 - triangle.v1 };
			pointToSide = { hitPoint - triangle.v1 };
			if (Vector3::Dot(triangle.normal, Vector3::Cross(side, pointToSide)) < 0) {
				return false;
			}

			//Side B
			side = { triangle.v0 - triangle.v2 };
			pointToSide = { hitPoint - triangle.v2 };
			if (Vector3::Dot(triangle.normal, Vector3::Cross(side, pointToSide)) < 0) {
				return false;
			}

			// Hit!
			if (ignoreHitRecord) {
				return true;
			}

			hitRecord.didHit = true;
			hitRecord.materialIndex = triangle.materialIndex;
			hitRecord.t = t;
			hitRecord.origin = hitPoint;
			hitRecord.normal = triangle.normal.Normalized();

			return true;
#elif defined(TRIANGLE_MT)
			// Möller-Trumbore Triangle Ray Intersection
			Vector3 edge1{ triangle.v1 - triangle.v0 };
			Vector3 edge2{ triangle.v2 - triangle.v0 };
			Vector3 pVec{ Vector3::Cross(ray.direction, edge2) };
			float InvDeterminant{ 1.0f / Vector3::Dot(edge1,pVec) };

			Vector3 tVec{ ray.origin - triangle.v0 };
			float baryU{ Vector3::Dot(tVec, pVec) * InvDeterminant };
			if (baryU < 0 || baryU > 1) {
				return false;
			}

			Vector3 qVec{ Vector3::Cross(tVec, edge1)};
			float baryV{ Vector3::Dot(ray.direction, qVec) * InvDeterminant };
			if (baryV < 0 || baryU + baryV > 1) {
				return false;
			}

			float t{ Vector3::Dot(edge2, qVec) * InvDeterminant };

			if (t >= ray.min && t <= ray.max) {
				if (ignoreHitRecord) {
					return true;
				}

				hitRecord.didHit = true;
				hitRecord.materialIndex = triangle.materialIndex;
				hitRecord.t = t;
				hitRecord.origin = ray.origin + ray.direction * t;
				hitRecord.normal = triangle.normal.Normalized();

				return true;
			}

			return false;
#endif
		}

		inline bool HitTest_Triangle(const Triangle& triangle, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_Triangle(triangle, ray, temp, true);
		}
#pragma endregion
#pragma region TriangeMesh HitTest

		inline bool SlabTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray) {
			float tx1 = (mesh.transformedMinAABB.x - ray.origin.x) / ray.direction.x;
			float tx2 = (mesh.transformedMaxAABB.x - ray.origin.x) / ray.direction.x;

			float tmin = std::min(tx1, tx2);
			float tmax = std::max(tx1, tx2);

			float ty1 = (mesh.transformedMinAABB.y - ray.origin.y) / ray.direction.y;
			float ty2 = (mesh.transformedMaxAABB.y - ray.origin.y) / ray.direction.y;

			tmin = std::max(tmin, std::min(ty1, ty2));
			tmax = std::min(tmax, std::max(ty1, ty2));

			float tz1 = (mesh.transformedMinAABB.z - ray.origin.z) / ray.direction.z;
			float tz2 = (mesh.transformedMaxAABB.z - ray.origin.z) / ray.direction.z;

			tmin = std::max(tmin, std::min(tz1, tz2));
			tmax = std::min(tmax, std::max(tz1, tz2));

			return tmin > 0 && tmax >= tmin;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray, HitRecord& hitRecord, bool ignoreHitRecord = false)
		{
			if (!SlabTest_TriangleMesh(mesh, ray)) {
				return false;
			}

			HitRecord tempHitRecord{};
			Triangle triangle{};

			for (int index{ 0 }; index < mesh.normals.size(); ++index) {
				triangle.cullMode = mesh.cullMode;
				triangle.normal = mesh.transformedNormals[index];
				triangle.materialIndex = mesh.materialIndex;
				triangle.v0 = mesh.transformedPositions[mesh.indices[(3 * index) + 0]];
				triangle.v1 = mesh.transformedPositions[mesh.indices[(3 * index) + 1]];
				triangle.v2 = mesh.transformedPositions[mesh.indices[(3 * index) + 2]];

				if (HitTest_Triangle(triangle, ray, tempHitRecord) && tempHitRecord.t < hitRecord.t) {
					hitRecord = tempHitRecord;
				}
			}
			return hitRecord.didHit;
		}

		inline bool HitTest_TriangleMesh(const TriangleMesh& mesh, const Ray& ray)
		{
			HitRecord temp{};
			return HitTest_TriangleMesh(mesh, ray, temp, true);
		}

#pragma endregion
	}

	namespace LightUtils
	{
		//Direction from target to light
		inline Vector3 GetDirectionToLight(const Light& light, const Vector3 origin)
		{
			return {light.origin - origin};
		}

		inline ColorRGB GetRadiance(const Light& light, const Vector3& target)
		{
			Vector3 lightDirection{ GetDirectionToLight(light, target) };

			switch (light.type) {
			case LightType::Point:
				return (light.color * light.intensity) / (lightDirection.SqrMagnitude());
				break;
			case LightType::Directional:
				return (light.color * light.intensity);
				break;
			}
		}
	}

	namespace Utils
	{
		//Just parses vertices and indices
#pragma warning(push)
#pragma warning(disable : 4505) //Warning unreferenced local function
		static bool ParseOBJ(const std::string& filename, std::vector<Vector3>& positions, std::vector<Vector3>& normals, std::vector<int>& indices)
		{
			std::ifstream file(filename);
			if (!file)
				return false;

			std::string sCommand;
			// start a while iteration ending when the end of file is reached (ios::eof)
			while (!file.eof())
			{
				//read the first word of the string, use the >> operator (istream::operator>>) 
				file >> sCommand;
				//use conditional statements to process the different commands	
				if (sCommand == "#")
				{
					// Ignore Comment
				}
				else if (sCommand == "v")
				{
					//Vertex
					float x, y, z;
					file >> x >> y >> z;
					positions.push_back({ x, y, z });
				}
				else if (sCommand == "f")
				{
					float i0, i1, i2;
					file >> i0 >> i1 >> i2;

					indices.push_back((int)i0 - 1);
					indices.push_back((int)i1 - 1);
					indices.push_back((int)i2 - 1);
				}
				//read till end of line and ignore all remaining chars
				file.ignore(1000, '\n');

				if (file.eof()) 
					break;
			}

			//Precompute normals
			for (uint64_t index = 0; index < indices.size(); index += 3)
			{
				uint32_t i0 = indices[index];
				uint32_t i1 = indices[index + 1];
				uint32_t i2 = indices[index + 2];

				Vector3 edgeV0V1 = positions[i1] - positions[i0];
				Vector3 edgeV0V2 = positions[i2] - positions[i0];
				Vector3 normal = Vector3::Cross(edgeV0V1, edgeV0V2);

				if(isnan(normal.x))
				{
					int k = 0;
				}

				normal.Normalize();
				if (isnan(normal.x))
				{
					int k = 0;
				}

				normals.push_back(normal);
			}

			return true;
		}
#pragma warning(pop)
	}
}