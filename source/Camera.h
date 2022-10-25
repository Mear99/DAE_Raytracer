#pragma once
#include <cassert>
#include <SDL_keyboard.h>
#include <SDL_mouse.h>

#include "Math.h"
#include "Timer.h"

namespace dae
{
	struct Camera
	{
		Camera() = default;

		Camera(const Vector3& _origin, float _fovAngle):
			origin{_origin},
			fovAngle{_fovAngle}
		{
		}


		Vector3 origin{};
		float fovAngle{90.f};

		Vector3 forward{Vector3::UnitZ};
		Vector3 up{Vector3::UnitY};
		Vector3 right{Vector3::UnitX};

		float totalPitch{0.f};
		float totalYaw{0.f};

		Matrix cameraToWorld{};

		const float movementSpeed{ 10 };
		const float rotationSpeed{ 5*TO_RADIANS };


		Matrix CalculateCameraToWorld()
		{
			Vector3 worldUp{ 0,1,0 };
			right = Vector3::Cross(worldUp, forward).Normalized();
			up = Vector3::Cross(forward, right);
			cameraToWorld = Matrix{ right, up, forward, origin };

			return cameraToWorld;
		}

		void Update(Timer* pTimer)
		{
			const float deltaTime = pTimer->GetElapsed();

			//Keyboard Input
			const uint8_t* pKeyboardState = SDL_GetKeyboardState(nullptr);

			//WASD movement
			if (pKeyboardState[SDL_SCANCODE_W]) {
				origin += pTimer->GetElapsed()*movementSpeed * forward;
			}
			if (pKeyboardState[SDL_SCANCODE_S]) {
				origin -= pTimer->GetElapsed() * movementSpeed * forward;
			}

			if (pKeyboardState[SDL_SCANCODE_D]) {
				origin += pTimer->GetElapsed() * movementSpeed * right;
			}
			if (pKeyboardState[SDL_SCANCODE_A]) {
				origin -= pTimer->GetElapsed() * movementSpeed * right;
			}



			//Mouse Input
			int mouseX{}, mouseY{};
			const uint32_t mouseState = SDL_GetRelativeMouseState(&mouseX, &mouseY);

			if ((SDL_BUTTON(1) & mouseState) != 0 && (SDL_BUTTON(3) & mouseState) != 0) {
				if (mouseY != 0) {
					origin -= pTimer->GetElapsed() * movementSpeed * up * (float(mouseY) / abs(mouseY));
				}
			}

			else if ((SDL_BUTTON(1) & mouseState) != 0) {
				totalYaw -= mouseX * rotationSpeed * pTimer->GetElapsed();

				if (mouseY != 0) {
					origin -= pTimer->GetElapsed() * movementSpeed * forward * (float(mouseY) / abs(mouseY));
				}
			}

			else if ((SDL_BUTTON(3) & mouseState) != 0) {
				totalYaw -= mouseX * rotationSpeed * pTimer->GetElapsed();
				totalPitch -= mouseY * rotationSpeed * pTimer->GetElapsed();
			}

			Matrix totalRotation{ Matrix::CreateRotation(totalPitch,totalYaw,0) };
			forward = totalRotation.TransformVector(Vector3::UnitZ);
			forward.Normalize();
		}
	};
}
