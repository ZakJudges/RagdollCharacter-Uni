#ifndef _ANIMATED_MESH_APP_H
#define _ANIMATED_MESH_APP_H

#include <system/application.h>
#include <graphics/sprite.h>
#include <maths/vector2.h>
#include <maths/vector4.h>
#include <maths/matrix44.h>
#include <vector>
#include "primitive_builder.h"

#include <chrono>

#ifdef _WIN32
// only on windows platforms
#include <platform/d3d11/input/touch_input_manager_d3d11.h>
#endif

// FRAMEWORK FORWARD DECLARATIONS
namespace gef
{
	class Platform;
	class SpriteRenderer;
	class Font;
	class Renderer3D;
	class Mesh;
	class InputManager;
	class MeshInstance;
}

class Character;

class AnimatedMeshApp : public gef::Application
{
public:
	AnimatedMeshApp(gef::Platform& platform);
	void Init();

	void CleanUp();
	bool Update(float frame_time);
	void Render();
	


	

private:
	void InitFont();
	void CleanUpFont();
	void DrawHUD();
	void SetupLights();
	void CreateBox(gef::Vector4 half_extents, gef::Vector4 translation);

	class Physics* physics_manager_;
	class Camera* camera_;
	

	gef::SpriteRenderer* sprite_renderer_;
	gef::Renderer3D* renderer_3d_;
	gef::InputManager* input_manager_;
	gef::Font* font_;
	PrimitiveBuilder* primitive_builder_;

	float fps_;
	bool display_controls_;
	float follow_strength_;

	gef::Vector2 mouse_position_;
	gef::Vector4 force_;

	Character* character_;
	gef::MeshInstance* ground_;
	std::vector<gef::MeshInstance*> boxes_;

	typedef std::chrono::high_resolution_clock clock;
	clock::time_point start, end;


};

#endif // _ANIMATED_MESH_APP_H
