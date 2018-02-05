#pragma once
#include "Shader.hpp"
#include "GraphicsEngine.hpp"
#include <utils/Geometry.hpp>
#include <vector>
#include <map>

namespace aitd {

	
struct MeshState {
	MeshState() {}
	MeshState(Shader::Ptr shader, uint64_t state, uint32_t fstencil,
			  uint32_t bstencil, uint8_t view) :
		m_shader(shader), m_state(state), m_fstencil(fstencil),
		m_bstencil(bstencil), m_viewId(view) {}	
	Shader::Ptr  m_shader;
	uint64_t m_state = BGFX_STATE_DEFAULT;
	uint32_t m_fstencil = BGFX_STENCIL_NONE;
	uint32_t m_bstencil = BGFX_STENCIL_NONE;
	uint8_t m_viewId = RENDER_PASS_GEOMETRY;	
};


struct StencilLayer {

	typedef std::shared_ptr<StencilLayer> Ptr;
	
	bgfx::VertexBufferHandle m_vbh;
	bgfx::IndexBufferHandle m_ibh;
	std::vector<PosColorVertex> m_vertices;
	std::vector<uint16_t> m_indices;
	std::vector<MeshState> m_render_passes;
	int m_depth_index;

	static StencilLayer::Ptr create(const std::vector<PosColorVertex>& vertices,
							   const std::vector<uint16_t>& indices,
							   const int depth_index) {
		StencilLayer::Ptr sl = StencilLayer::Ptr(new StencilLayer());
		const bgfx::Memory* mem;
				
		sl->m_vertices = vertices;
		sl->m_indices = indices;
		sl->m_depth_index = depth_index;

		/////////////////
		//
		// 2. in the bgmask phase, the stencil gets the mask if it fits the mesh index
		//
		//////////////////
		
		// two passes. One to set the meshes and another to set the masks		
		sl->m_render_passes = {
			// {
			// 	Shader::Ptr(new Shader("vs_default", "fs_default")),
			// 	| BGFX_STATE_ALPHA_WRITE
			// 	| BGFX_STATE_DEPTH_WRITE
			// 	| BGFX_STATE_DEPTH_TEST_LESS
			// 	| BGFX_STATE_MSAA,
			// 	// fstencil
			// 	BGFX_STENCIL_TEST_EQUAL
			// 	| BGFX_STENCIL_FUNC_REF(depth_index)
			// 	| BGFX_STENCIL_FUNC_RMASK(0xff)
			// 	| BGFX_STENCIL_OP_FAIL_S_ZERO
			// 	| BGFX_STENCIL_OP_FAIL_Z_REPLACE
			// 	| BGFX_STENCIL_OP_PASS_Z_REPLACE,
			// 	// bstencil
			// 	BGFX_STENCIL_NONE,
			// 	RENDER_PASS_PRE_BGMASK
			//},
			{
				Shader::Ptr(new Shader("vs_default", "fs_default")),
				0
				//| BGFX_STATE_RGB_WRITE
				// | BGFX_STATE_ALPHA_WRITE				
				// | BGFX_STATE_DEPTH_WRITE
				// | BGFX_STATE_DEPTH_TEST_LESS
				| BGFX_STATE_MSAA,
				// fstencil
				BGFX_STENCIL_TEST_EQUAL // in the bgmask the 
				| BGFX_STENCIL_FUNC_REF(depth_index)
				| BGFX_STENCIL_FUNC_RMASK(0xff)
				| BGFX_STENCIL_OP_FAIL_S_ZERO
				| BGFX_STENCIL_OP_FAIL_Z_REPLACE
				| BGFX_STENCIL_OP_PASS_Z_REPLACE,
				// bstencil
				BGFX_STENCIL_NONE,
				RENDER_PASS_BGMASK
			}
		};


		for (auto pass : sl->m_render_passes) {
			pass.m_shader->init();	   	
		}
		
		std::map<int, uint32_t> color_map = {
			{0, 0xff000000 },
			{1, 0xff0000ff },
			{2, 0xff000ff0 },
			{3, 0xff00ff00 },
			{4, 0xff0ff000 },
			{5, 0xffff0000 },
			{6, 0xffaa00f0 },
			{7, 0xffbb00f0 }
		};

		// make a color map with the depth indices
		for (auto& v : sl->m_vertices) {
			v.m_abgr = color_map[depth_index];
		}
		
		//TODO: put this in destructor of StencilLayer!
		if (bgfx::isValid(sl->m_vbh) )
		{
			bgfx::destroy(sl->m_vbh);
		}

		mem = bgfx::makeRef(&sl->m_vertices[0], sizeof(PosColorVertex) * sl->m_vertices.size());
		sl->m_vbh = bgfx::createVertexBuffer(mem, PosColorVertex::ms_decl);

		if (bgfx::isValid(sl->m_ibh) )
		{
			bgfx::destroy(sl->m_ibh);
		}
		
		mem = bgfx::makeRef(&sl->m_indices[0], sizeof(uint16_t) * sl->m_indices.size());
		sl->m_ibh = bgfx::createIndexBuffer(mem);
		return sl;
	}
};

//TODO: rename this to stencil and move to Stencil.hpp
class BgMask {
public:
	typedef std::pair<int, Geometry::Polygon<Vec2i>> IndexPolygon;
	typedef std::shared_ptr<BgMask> Ptr;
	BgMask(const std::vector<IndexPolygon>&);
	void craftStencilBuffer(const Geometry::Polygon<Vec2i>& mesh_poly);
	void craftStencilBuffer(const std::vector<IndexPolygon>& polys);
	void setStencilIndex(int idx);
	void submit();
	void clearMask();
 
	std::vector<IndexPolygon> m_polygons;
	std::vector<StencilLayer::Ptr> m_layers;
};

class Mesh {
public:
	typedef std::shared_ptr<Mesh> Ptr;	
	Mesh() {}
	void submit(const float* mtx);

	
	bgfx::DynamicVertexBufferHandle m_dvbh;
	bgfx::DynamicIndexBufferHandle m_dibh;
	std::vector<MeshState> m_render_passes;	
};

 }
