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

using namespace dae;

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
	const Matrix cameraToWorld{ camera.CalculateCameraToWorld() };
	const float lightCheckOffset{ 0.001f };

	for (int px{}; px < m_Width; ++px)
	{
		for (int py{}; py < m_Height; ++py)
		{

			Vector3 rayDirection{};
			rayDirection.x = ((2 * (px + 0.5f) / m_Width) - 1) * aspectRatio*fov;
			rayDirection.y = (1 - (2 * (py + 0.5f) / m_Height))*fov;
			rayDirection.z = 1;
			rayDirection.Normalize();

			rayDirection = cameraToWorld.TransformVector(rayDirection);

			Ray hitRay{ camera.origin, rayDirection };

			ColorRGB finalColor{0,0,0};
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
						Vector3 startPoint{ closestHit.origin + closestHit.normal * lightCheckOffset };
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
	}

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
