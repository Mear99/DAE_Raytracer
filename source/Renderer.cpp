//External includes
#include "SDL.h"
#include "SDL_surface.h"

//Project includes
#include "Renderer.h"
#include "Math.h"
#include "Matrix.h"
#include "Material.h"
#include "Scene.h"
#include "Utils.h"

#include <future>
#include <ppl.h>

using namespace dae;

//#define ASYNC
#define PARALLEL


Renderer::Renderer(SDL_Window * pWindow) :
	m_pWindow(pWindow),
	m_pBuffer(SDL_GetWindowSurface(pWindow))
{
	//Initialize
	SDL_GetWindowSize(pWindow, &m_Width, &m_Height);
	m_pBufferPixels = static_cast<uint32_t*>(m_pBuffer->pixels);
}

void Renderer::Render(Scene* pScene) const
{
	Camera& camera = pScene->GetCamera();
	auto& materials = pScene->GetMaterials();
	auto& lights = pScene->GetLights();

	const float aspectRatio{ float(m_Width) / m_Height };
	const float fov{ tanf(camera.fovAngle * TO_RADIANS/2) };
	camera.CalculateCameraToWorld();

	const uint32_t numOfPixels{ uint32_t(m_Width * m_Height) };

#if defined(ASYNC)
	// Async execution
	const uint32_t numOfCores{ std::thread::hardware_concurrency() };
	std::vector<std::future<void>> asyncFutures{};
	const uint32_t numPixelPerTask{ numOfPixels / numOfCores };
	uint32_t numUnassignedPixels{ numOfPixels % numOfCores };
	uint32_t currentPixelIndex{ 0 };

	for (uint32_t coreId{ 0 }; coreId < numOfCores; ++coreId) {

		uint32_t taskSize{ numPixelPerTask };
		if (numUnassignedPixels > 0) {
			++taskSize;
			--numUnassignedPixels;
		}

		asyncFutures.push_back(std::async(std::launch::async, [=, this] {

			const uint32_t pixelIndexEnd = currentPixelIndex + taskSize;
			for (uint32_t pixelIndex{ currentPixelIndex }; pixelIndex < pixelIndexEnd; ++pixelIndex) {
				RenderPixel(pScene, pixelIndex, fov, aspectRatio, camera, lights, materials);
			}

		}));

		currentPixelIndex += taskSize;
	}

	for (const std::future<void>& f : asyncFutures) {
		f.wait();
	}

#elif defined(PARALLEL)
	// Parallel for execution
	concurrency::parallel_for(0u, numOfPixels, [=, this](int i) {
		RenderPixel(pScene, i, fov, aspectRatio, camera, lights, materials);
	});

#else
	// Synchronous execution
	for (uint32_t pixel{ 0 }; pixel < numOfPixels; ++pixel) {
		RenderPixel(pScene, pixel, fov, aspectRatio, camera, lights, materials);
	}

#endif

	//@END
	//Update SDL Surface
	SDL_UpdateWindowSurface(m_pWindow);
}

bool Renderer::SaveBufferToImage() const
{
	return SDL_SaveBMP(m_pBuffer, "RayTracing_Buffer.bmp");
}

void Renderer::CycleLightingMode() {
	m_CurrentLightingMode = LightingMode((int(m_CurrentLightingMode) + 1) % 4);
}

void Renderer::RenderPixel(Scene* pScene, uint32_t pixelIndex, float fov, float aspectRatio, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const {
	
	const int px = pixelIndex % m_Width;
	const int py = pixelIndex / m_Width;

	Vector3 rayDirection{};
	rayDirection.x = ((2 * (px + 0.5f) / m_Width) - 1) * aspectRatio * fov;
	rayDirection.y = (1 - (2 * (py + 0.5f) / m_Height)) * fov;
	rayDirection.z = 1;
	rayDirection.Normalize();

	rayDirection = camera.cameraToWorld.TransformVector(rayDirection);

	Ray hitRay{ camera.origin, rayDirection };

	ColorRGB finalColor{ 0,0,0 };
	HitRecord closestHit{};

	pScene->GetClosestHit(hitRay, closestHit);

	if (closestHit.didHit) {
		for (const Light& light : lights) {

			// Vector from hit to light
			Vector3 toLightDirection{ LightUtils::GetDirectionToLight(light, closestHit.origin) };
			float distanceToLight{ toLightDirection.Magnitude() };
			toLightDirection.Normalize();

			// Outgoing light direction (depends on light type)
			Vector3 lightDirection{ light.direction };
			if (light.type == LightType::Point) {
				lightDirection = toLightDirection;
			}

			// Cosine Law
			float cosineLaw{ Vector3::Dot(closestHit.normal, lightDirection) };

			// Light hits the surface
			if (cosineLaw >= 0) {

				// Illumination is direct (so nothing between surface and light) or shadows are ignored
				Vector3 startPoint{ closestHit.origin + closestHit.normal * 0.001f };
				Ray toLight{ startPoint, toLightDirection };
				toLight.max = distanceToLight;
				if (!pScene->DoesHit(toLight) || !m_ShadowsEnabled) {

					// Radiance
					ColorRGB radiance{ LightUtils::GetRadiance(light,closestHit.origin) };

					// BRDF color
					ColorRGB BRDFColor{ materials[closestHit.materialIndex]->Shade(closestHit, toLightDirection, -hitRay.direction) };

					switch (m_CurrentLightingMode) {
					case LightingMode::ObservedArea:
						finalColor += ColorRGB{ cosineLaw, cosineLaw, cosineLaw };
						break;

					case LightingMode::Radiance:
						finalColor += radiance;
						break;

					case LightingMode::BRDF:
						finalColor += BRDFColor;
						break;

					case LightingMode::Combined:
						finalColor += radiance * BRDFColor * cosineLaw;
						break;
					}
				}
			}
		}
	}

	//Update Color in Buffer
	finalColor.MaxToOne();

	m_pBufferPixels[px + (py * m_Width)] = SDL_MapRGB(m_pBuffer->format,
		static_cast<uint8_t>(finalColor.r * 255),
		static_cast<uint8_t>(finalColor.g * 255),
		static_cast<uint8_t>(finalColor.b * 255));

}

