#include "animated_mesh_app.h"
#include <system/platform.h>
#include <graphics/sprite_renderer.h>
#include <graphics/texture.h>
#include <graphics/mesh.h>
#include <graphics/mesh_instance.h>
#include <graphics/primitive.h>
#include <assets/png_loader.h>
#include <graphics/image_data.h>
#include <graphics/font.h>
#include <maths/vector2.h>
#include <input/input_manager.h>
#include <input/sony_controller_input_manager.h>
#include <input/keyboard.h>
#include <maths/math_utils.h>
#include <graphics/renderer_3d.h>
#include "btBulletDynamicsCommon.h"
#include "Physics.h"
#include "Camera.h"
#include <system/debug_log.h>

#include "Character.h"


AnimatedMeshApp::AnimatedMeshApp(gef::Platform& platform) :
	Application(platform),
	sprite_renderer_(NULL),
	font_(NULL),
	renderer_3d_(NULL),
	primitive_builder_(NULL),
	camera_(NULL),
	ground_(NULL),
	character_(NULL)
{
	display_controls_ = false;
}

void AnimatedMeshApp::Init()
{
	sprite_renderer_ = gef::SpriteRenderer::Create(platform_);
	renderer_3d_ = gef::Renderer3D::Create(platform_);
	input_manager_ = gef::InputManager::Create(platform_);
	primitive_builder_ = new PrimitiveBuilder(platform_);

	physics_manager_ = new Physics();
	physics_manager_->Init();

	camera_ = new Camera(platform_);
	camera_->Init(gef::Vector4(0.0f, 1.0f, 7.0f), gef::Vector4(0.0f, 0.0f, 0.0f), gef::Vector4(0.0f, 1.0f, 0.0f), gef::DegToRad(45.0f), 0.01f, 100.0f);
	
	InitFont();
	SetupLights();

	mouse_position_ = gef::Vector2(0.0f, 0.0f);

	ground_ = new gef::MeshInstance();
	gef::Vector4 ground_half_extents(20, 1, 20);
	ground_->set_mesh(primitive_builder_->CreateBoxMesh(ground_half_extents, gef::Vector4(0, 0, 0)));
	gef::Matrix44 ground_transform;
	ground_transform.SetIdentity();
	ground_transform.SetTranslation(gef::Vector4(0.0f, -1.25f, 0.0f));
	ground_->set_transform(ground_transform);
	physics_manager_->CreateStaticObject(ground_half_extents, ground_transform);

	CreateBox(gef::Vector4(0.5f, 0.3f, 0.25f), gef::Vector4(0.0f, -0.2f, -2.0f));
	//CreateBox(gef::Vector4(0.25f, 0.5f, 0.75f), gef::Vector4(-2.5f, 0.0f, 0.0f));
	//CreateBox(gef::Vector4(1.0f, 0.5f, 0.25f), gef::Vector4(-1.5f, 0.0f, 1.0f));


	character_ = new Character(platform_, physics_manager_->GetWorld());
	character_->Init();
}

void AnimatedMeshApp::CreateBox(gef::Vector4 box_half_extents, gef::Vector4 translation)
{
	gef::MeshInstance* box = new gef::MeshInstance();
	gef::Matrix44 box_transform;
	box_transform.SetIdentity();
	box->set_mesh(primitive_builder_->CreateBoxMesh(box_half_extents, gef::Vector4(0.0f, 0.0f, 0.0f)));
	box_transform.SetIdentity();
	box_transform.SetTranslation(translation);
	box->set_transform(box_transform);
	physics_manager_->CreateStaticObject(box_half_extents, box_transform);
	boxes_.push_back(box);
}

void AnimatedMeshApp::CleanUp()
{
	CleanUpFont();

	delete input_manager_;
	input_manager_ = NULL;

	delete sprite_renderer_;
	sprite_renderer_ = NULL;

	delete renderer_3d_;
	renderer_3d_ = NULL;
}

bool AnimatedMeshApp::Update(float frame_time)
{
	///start = clock::now();

	fps_ = 1.0f / frame_time;

	if (physics_manager_)
	{
		physics_manager_->Update(frame_time);
	}

	if (character_)
	{
		character_->Update(frame_time);
		follow_strength_ = character_->GetFollowStrength();
	}
	

	// read input devices
	if (input_manager_)
	{
		input_manager_->Update();

		// keyboard input
		gef::Keyboard* keyboard = input_manager_->keyboard();
		if (keyboard)
		{
			// Calculate the new lookat direction.
			gef::Vector4 new_lookat = camera_->GetCameraLookat() - camera_->GetCameraEye();
			new_lookat.Normalise();
			gef::Vector4 left(0.0f, 1.0f, .0f);
			left = left.CrossProduct(new_lookat);
			left.Normalise();

			//	Camera controls.
			if (keyboard->IsKeyDown(gef::Keyboard::KeyCode::KC_W))
			{
				camera_->SetCameraEye(camera_->GetCameraEye() + (new_lookat * frame_time * 5.0f));
			}
			if (keyboard->IsKeyDown(gef::Keyboard::KeyCode::KC_S))
			{
				camera_->SetCameraEye(camera_->GetCameraEye() + (new_lookat * frame_time * -5.0f));
			}
			if (keyboard->IsKeyDown(gef::Keyboard::KeyCode::KC_A))
			{
				camera_->SetCameraEye(camera_->GetCameraEye() + (left * frame_time * 5.0f));
			}
			if (keyboard->IsKeyDown(gef::Keyboard::KeyCode::KC_D))
			{
				camera_->SetCameraEye(camera_->GetCameraEye() + (-left * frame_time * 5.0f));
			}

			//	Look at the character.
			if (keyboard->IsKeyDown(gef::Keyboard::KeyCode::KC_LSHIFT))
			{
				camera_->SetCameraLookat(character_->GetPosition());
			}
			else
			{
				camera_->SetCameraLookat(camera_->GetCameraEye() + gef::Vector4(0.0f, 0.0f, -1.0f));
			}

			//	Toggles.
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_4))
			{
				character_->ToggleMesh();
			}	
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_2))
			{
				character_->ToggleSkeleton();
			}
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_3))
			{
				character_->ToggleRagdoll();
			}
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_1))
			{
				character_->ToggleActiveRagdoll();
			}
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_C))
			{
				if (display_controls_)
				{
					display_controls_ = false;
				}
				else
				{
					display_controls_ = true;
				}
			}

			//	Reset camera and character.
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_LCONTROL))
			{
				camera_->SetCameraEye(gef::Vector4(0.0f, 1.0f, 7.0f));
				character_->Reset();
			}

			//	Character controls.
			if (keyboard->IsKeyDown(gef::Keyboard::KeyCode::KC_COMMA))
			{
				character_->Rotate(false);
			}
			if (keyboard->IsKeyDown(gef::Keyboard::KeyCode::KC_PERIOD))
			{
				character_->Rotate(true);
			}
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_SPACE))
			{
				character_->StandUp();
			}
			//	Apply a force to the character.
			gef::Vector4 direction(0.0f, 0.0f, 0.0f);
			bool apply_force = false;
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_UP))
			{
				direction = new_lookat;
				apply_force = true;
			}
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_DOWN))
			{
				direction = -new_lookat;
				apply_force = true;
			}
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_LEFT))
			{
				direction = left;
				apply_force = true;
			}
			if (keyboard->IsKeyPressed(gef::Keyboard::KeyCode::KC_RIGHT))
			{
				direction = -left;
				apply_force = true;
			}
			if (apply_force)
			{
				direction.Normalise();
				character_->ApplyForce(direction);
			}
		}
	}

	///clock::time_point end = clock::now();
	///auto time_taken = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
	///gef::DebugOut("Character update took (us): ", gef::Vector4(time_taken, 0.0f, 0.0f));
	
	return true;
}

void AnimatedMeshApp::Render()
{
	renderer_3d_->set_projection_matrix(camera_->GetProjectionMatrix());
	renderer_3d_->set_view_matrix(camera_->GetViewMatrix());

	// draw meshes here
	renderer_3d_->Begin();

	//	Render the componenents that make up the character.
	character_->Render(renderer_3d_);
	
	if (ground_)
	{
		renderer_3d_->set_override_material(&primitive_builder_->green_material());
		renderer_3d_->DrawMesh(*ground_);
	}

	renderer_3d_->set_override_material(nullptr);
	for(int i = 0; i < boxes_.size(); i++)
	{
		if(boxes_[i])
		{
			renderer_3d_->DrawMesh(*boxes_[i]);
		}
	}

	renderer_3d_->End();

	// setup the sprite renderer, but don't clear the frame buffer
	// draw 2D sprites here
	sprite_renderer_->Begin(false);
	DrawHUD();
	sprite_renderer_->End();
}

void AnimatedMeshApp::InitFont()
{
	font_ = new gef::Font(platform_);
	font_->Load("comic_sans");
}

void AnimatedMeshApp::CleanUpFont()
{
	delete font_;
	font_ = NULL;
}

void AnimatedMeshApp::DrawHUD()
{
	if(font_)
	{
		// display frame rate
		//font_->RenderText(sprite_renderer_, gef::Vector4(850.0f, 510.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, "FPS: %.1f", fps_);
		font_->RenderText(sprite_renderer_, gef::Vector4(platform_.width() - 115.0f, platform_.height() - 30.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, "FPS: %.1f", fps_);

		//font_->RenderText(sprite_renderer_, gef::Vector4(730.0f, 485.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, "Follow Strength: %.1f", follow_strength_);
		font_->RenderText(sprite_renderer_, gef::Vector4(platform_.width() - 215.0f ,platform_.height() - 55.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, "Follow Strength: %.1f", follow_strength_);


		char* message;
		if (!display_controls_)
		{
			message = "Display Controls (C)";
			font_->RenderText(sprite_renderer_, gef::Vector4(0.0f, 350.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
		}
		else
		{
			message = "Render Toggles:";
			font_->RenderText(sprite_renderer_, gef::Vector4(0.0f, 390.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "Active Ragdoll (1)";
			font_->RenderText(sprite_renderer_, gef::Vector4(0.0f, 425.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "Ragdoll Skeleton (2)";
			font_->RenderText(sprite_renderer_, gef::Vector4(0.0f, 450.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "Ragdoll (3)";
			font_->RenderText(sprite_renderer_, gef::Vector4(0.0f, 475.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "Skinned Mesh (4)";
			font_->RenderText(sprite_renderer_, gef::Vector4(0.0f, 500.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);

			message = "Character controls:";
			font_->RenderText(sprite_renderer_, gef::Vector4(275.0f, 390.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "Stand up (SPACE)";
			font_->RenderText(sprite_renderer_, gef::Vector4(275.0f, 425.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "Rotate (<, >)";
			font_->RenderText(sprite_renderer_, gef::Vector4(275.0f, 450.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "Apply force (ARROWS)";
			font_->RenderText(sprite_renderer_, gef::Vector4(275.0f, 475.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);

			message = "Camera controls:";
			font_->RenderText(sprite_renderer_, gef::Vector4(550.0f, 390.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "Movement (W,S,A,D)";
			font_->RenderText(sprite_renderer_, gef::Vector4(550.0f, 425.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "Look at character (LSHIFT)";
			font_->RenderText(sprite_renderer_, gef::Vector4(550.0f, 450.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
			message = "RESET (CTRL)";
			font_->RenderText(sprite_renderer_, gef::Vector4(550.0f, 475.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);

			message = "Hide Controls (C)";
			font_->RenderText(sprite_renderer_, gef::Vector4(0.0f, 350.0f, -0.9f), 1.0f, 0xffffffff, gef::TJ_LEFT, message);
		}
	}
}

void AnimatedMeshApp::SetupLights()
{
	gef::PointLight default_point_light;
	default_point_light.set_colour(gef::Colour(0.7f, 0.7f, 1.0f, 1.0f));
	default_point_light.set_position(gef::Vector4(-300.0f, -500.0f, 100.0f));

	gef::Default3DShaderData& default_shader_data = renderer_3d_->default_shader_data();
	default_shader_data.set_ambient_light_colour(gef::Colour(0.5f, 0.5f, 0.5f, 1.0f));
	default_shader_data.AddPointLight(default_point_light);
}

