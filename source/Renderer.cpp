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

			ColorRGB finalColor{};
			HitRecord closestHit{};

			pScene->GetClosestHit(hitRay, closestHit);

			if (closestHit.didHit) {

				// Material color
				finalColor = materials[closestHit.materialIndex]->Shade();

				// Hard shadows
				for (const Light& light : lights) {

					Vector3 startPoint{ closestHit.origin + closestHit.normal * lightCheckOffset };
					Vector3 toLightDirection{ LightUtils::GetDirectionToLight(light, closestHit.origin)};

					Ray toLight{ startPoint, toLightDirection.Normalized()};
					toLight.max = toLightDirection.Magnitude();

					if (pScene->DoesHit(toLight)) {
						finalColor *= 0.5f;
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
