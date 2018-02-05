#pragma once

#include "GraphicsEngine.hpp"
#include "Mesh.hpp"
#include <World.hpp>
#include <Actor.hpp>
#include <entities/Entity.hpp>
#include <entities/System.hpp>
#include <utils/Geometry.hpp>

namespace aitd {

	
class CameraComponent {
public:
	CameraComponent(const Vec3f& camera_pos,
					const Eigen::Matrix4f& camera_proj,
					const Eigen::Matrix4f& camera_view) : cam_pos(camera_pos) {

		//initialize matrices
		memcpy(view, camera_view.data(), sizeof(float)*16);
		memcpy(proj, camera_proj.data(), sizeof(float)*16);		
		m_camera.init();
	}

	Eigen::Matrix4f getVPMatrix() {
		Eigen::Matrix4f V(view);
		Eigen::Matrix4f P(proj);
		return P * V;
	}

	void render(float delta) {

		//m_camera.mtxLookAt(view);
		bgfx::setViewTransform(RENDER_PASS_GEOMETRY, view, proj);
		bgfx::setViewTransform(RENDER_PASS_PRE_BGMASK, view, proj);

//		bgfx::setViewTransform(RENDER_PASS_BGMASK, view, proj);
//		m_camera.update(delta);
	}

	Vec3f cam_pos;
	float view[16];
	float proj[16];
	Camera m_camera;
};

// class BgMaskComponent {
// public:

// 	BgMaskComponent() {

// 	}
	
// 	void render(float delta) {
// 		bgfx::setTexture(0, texture_uniform, texture);
// 		bgfx::setState(0
// 					   | BGFX_STATE_RGB_WRITE
// 					   | BGFX_STATE_ALPHA_WRITE);
// 		screenSpaceQuad((float)GraphicsEngine::WIDTH, (float)GraphicsEngine::HEIGHT, false, 1.0f, 1.0f);
// 		bgfx::submit(RENDER_PASS_BGMASK, program->getHandle());
// 	}

// protected:

// 	//vertex buffer
	
// }

class BgImageComponent {
public:
	BgImageComponent(unsigned char* buffer) {

		const int num_pixels = GraphicsEngine::WIDTH * GraphicsEngine::HEIGHT;
		const bgfx::Memory* mem = bgfx::alloc(num_pixels*3);
		memcpy((unsigned char*)mem->data, buffer, num_pixels*3);
		const uint32_t flags = 0
	 	| BGFX_TEXTURE_U_CLAMP
	 	| BGFX_TEXTURE_V_CLAMP
	 	| BGFX_TEXTURE_MIN_POINT
	 	| BGFX_TEXTURE_MAG_POINT;
		texture = bgfx::createTexture2D(GraphicsEngine::WIDTH, GraphicsEngine::HEIGHT, false, 1,
										bgfx::TextureFormat::RGB8, flags, mem);	
		program = Shader::Ptr(new Shader("vs_backg", "fs_backg"));
		program->init();
	}

	void render(float delta) {
		bgfx::setTexture(0, texture_uniform, texture);
		bgfx::setState(0
					   | BGFX_STATE_RGB_WRITE
//					   | BGFX_STATE_DEPTH_WRITE
//					   | BGFX_STATE_DEPTH_TEST_LESS
//					   | BGFX_STATE_CULL_CW
					   | BGFX_STATE_MSAA);
					   
					   // | BGFX_STATE_RGB_WRITE
					   // | BGFX_STATE_ALPHA_WRITE
					   // | BGFX_STATE_DEPTH_TEST_LESS);
		
		bgfx::setStencil( BGFX_STENCIL_NONE,//TEST_ALWAYS,
		// 				  | BGFX_STENCIL_OP_FAIL_S_KEEP
		// 				  | BGFX_STENCIL_OP_FAIL_Z_KEEP
		// 				  | BGFX_STENCIL_OP_PASS_Z_KEEP,
		 				  BGFX_STENCIL_NONE);

		screenSpaceQuad((float)GraphicsEngine::WIDTH, (float)GraphicsEngine::HEIGHT, false, 1.0f, 1.0f);
		bgfx::submit(RENDER_PASS_BACKGROUND, program->getHandle());
	}
	
protected:

	Shader::Ptr program;
	bgfx::TextureHandle texture;
	bgfx::UniformHandle texture_uniform;
};

class DebugComponent {
public:
	DebugComponent(Geometry::DebugMesh::Ptr d) : debug_mesh(d) {}

	void render(float delta) {	
		debug_mesh->render(delta);
	}
protected:

	Geometry::DebugMesh::Ptr debug_mesh;
};

class MeshComponent {

public:

	MeshComponent(Actor::Ptr actor) {
		offsets = actor->offsets;
		vertices = actor->vertices;
		primitives = actor->primitives;
		skeleton = actor->skeleton->deepCopy();
		generateMesh();

		for (int i = 0; i < 4; i++) {
			mask_value[i] = 0;
		}
	}

	//TODO: put this in the system to have components contain no logic at all
	void render(float delta, const float* mtx);
	void generateMesh();
	void updateVertices();
	void setDepthIndex(int di) {
		depth_index = di;

		mesh->m_render_passes[0].m_fstencil =
			BGFX_STENCIL_TEST_ALWAYS
			| BGFX_STENCIL_FUNC_REF(depth_index)
			| BGFX_STENCIL_FUNC_RMASK(0xff)
			| BGFX_STENCIL_OP_FAIL_S_REPLACE
			| BGFX_STENCIL_OP_FAIL_Z_KEEP
			| BGFX_STENCIL_OP_PASS_Z_KEEP;
		
		mesh->m_render_passes[1].m_fstencil = //BGFX_STENCIL_NONE;
			BGFX_STENCIL_TEST_GREATER//GREATER show if the depth_index is more than the stencil
			| BGFX_STENCIL_FUNC_REF(depth_index)
			| BGFX_STENCIL_FUNC_RMASK(0xff)
			| BGFX_STENCIL_OP_FAIL_S_KEEP
			| BGFX_STENCIL_OP_FAIL_Z_KEEP
			| BGFX_STENCIL_OP_PASS_Z_KEEP;
	}
	void updateSkeleton(Skeleton::Ptr skel) {
		skeleton = skel;
		updateVertices();
	}

	// void craftStencil(const std::vector<Geometry::Polygon<Vec2i> >& plist) {
	// 	mask->craftStencilBuffer(plist);
	// }


	Geometry::Polygon<Eigen::Vector2i> getProjectedMask(const Eigen::Matrix4f& MVP) {

		std::vector<Eigen::Vector2i> mask;
		//project vertices in screen
		for (int i = 0; i < vertices.size(); i++) {
			Eigen::Matrix<float, 4, 1> point = MVP * Eigen::Matrix<float, 4, 1>(vertices[i](0), vertices[i](1), vertices[i](2), 1.0);
			point /= point(3);

			int winX = (1+point(0))* GraphicsEngine::WIDTH/2.0f;
			int winY = (1+point(1))* GraphicsEngine::HEIGHT/2.0f;
			
			mask.push_back({winX, winY});
		}

		// compute polygon from the scatter points
		return Geometry::computePolygonFromScatter(mask);			  
	}
	
	
protected:

	//current bg mask for this mesh
	//BgMask::Ptr mask;
	
	//mesh for rendering
	Mesh::Ptr mesh;

	//offsets from bone locations. Fixed since loading time.
	std::vector<Eigen::Vector3f> offsets; 

	//includes vertices and bones. Is modified when the mesh animates
	std::vector<Eigen::Vector3f> vertices; 
	std::vector<uint16_t> indices;
	
	//vector of polygons pointing to the vertex buffer
	std::vector<Primitive::Ptr> primitives;

	//skeleton that defines the mesh deformation
	Skeleton::Ptr skeleton;

	// value to write during pre-mask phase
	bgfx::UniformHandle u_mask_value;
	float mask_value[4];

	int depth_index = -1;

};

class RenderSystem : public System<RenderSystem> {
public:

	RenderSystem(World::Ptr w) : world(w) {}
	void update(EntityManager & em, EventManager &evm, float delta);

protected:

	World::Ptr world;
};
}
