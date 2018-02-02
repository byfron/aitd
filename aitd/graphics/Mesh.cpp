#include "Mesh.hpp"

namespace aitd {
void BgMask::render() {

}
	

// different views/states for a mesh


// each mesh has 2 different render passes!!!
// 1. create stencil buffer (if necessary)
// 2. render mesh


void Mesh::submit() {

	// stencil masking
	
	for (auto pass : m_render_passes) {		
		bgfx::setIndexBuffer(m_dibh);
		bgfx::setVertexBuffer(0, m_dvbh);
		bgfx::setState(pass.m_state);		
		bgfx::submit(pass.m_viewId,
					 pass.m_shader->getHandle());
	}
}
	
}
