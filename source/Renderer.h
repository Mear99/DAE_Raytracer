#pragma once

#include <cstdint>
#include <vector>
#include "Camera.h"
#include "Material.h"
#include "DataTypes.h"

struct SDL_Window;
struct SDL_Surface;

namespace dae
{
	class Scene;

	class Renderer final
	{
	public:
		Renderer(SDL_Window* pWindow);
		~Renderer() = default;

		Renderer(const Renderer&) = delete;
		Renderer(Renderer&&) noexcept = delete;
		Renderer& operator=(const Renderer&) = delete;
		Renderer& operator=(Renderer&&) noexcept = delete;

		void Render(Scene* pScene) const;
		void RenderPixel(Scene* pScene, uint32_t pixelIndex, float fov, float aspectRatio, const Camera& camera, const std::vector<Light>& lights, const std::vector<Material*>& materials) const;

		bool SaveBufferToImage() const;

		void ToggleShadows() { m_ShadowsEnabled = !m_ShadowsEnabled; }
		void CycleLightingMode();

	private:
		SDL_Window* m_pWindow{};

		SDL_Surface* m_pBuffer{};
		uint32_t* m_pBufferPixels{};

		int m_Width{};
		int m_Height{};

		bool m_ShadowsEnabled{ true };

		enum class LightingMode { ObservedArea, Radiance, BRDF, Combined };
		LightingMode m_CurrentLightingMode{ LightingMode::Combined };
	};
}
