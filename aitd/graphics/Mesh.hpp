#pragma once
#include "Shader.hpp"
#include "GraphicsEngine.hpp"
#include <utils/Geometry.hpp>
#include <vector>

namespace aitd {

struct MeshState {
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

class BgMask {
public:
	typedef std::shared_ptr<BgMask> Ptr;
	void render();	
	void submit();
	bgfx::DynamicVertexBufferHandle m_dvbh;
	bgfx::DynamicIndexBufferHandle m_dibh;
	std::vector<Geometry::Polygon<Vec2i>> polygon_list;
};

class Mesh {
public:
	typedef std::shared_ptr<Mesh> Ptr;	
	Mesh() {}
	void submit();	
	bgfx::DynamicVertexBufferHandle m_dvbh;
	bgfx::DynamicIndexBufferHandle m_dibh;
	std::vector<MeshState> m_render_passes;	
};

}
