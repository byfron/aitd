#include "Mesh.hpp"
#include <bgfx/bgfx.h>
#include <graphics/clip2tri/clip2tri.h>

namespace aitd {

int makeTriangleStrip(const std::vector<Vec2i> &polygon,
					  std::vector<PosColorVertex> & triangles,
					  std::vector<uint16_t> & indices) {

	if (polygon.size() == 0) {
		assert(false);
		return 0;
	}

	int vertex_count = 0;

	std::vector< std::vector<c2t::Point> > inputPolygons;
	std::vector<c2t::Point> outputTriangles;
	std::vector<c2t::Point> boundingPolygon;

	std::vector<c2t::Point> poly;
	for (auto p : polygon) {
		poly.push_back(c2t::Point(p(0), p(1)));
	}

	inputPolygons.push_back(poly);

	c2t::clip2tri clip2tri;
	clip2tri.triangulate(inputPolygons, outputTriangles, boundingPolygon);

	// for (size_t i = 0; i < outputTriangles.size(); i+=3) {

	// 	std::vector<Vec3f> points;
	// 	points.push_back(Vec3f(outputTriangles[i].x, outputTriangles[i].y, 0));
	// 	points.push_back(Vec3f(outputTriangles[i+1].x, outputTriangles[i+1].y, 0));
	// 	points.push_back(Vec3f(outputTriangles[i+2].x, outputTriangles[i+2].y, 0));

	// 	DebugManager::push_polygon(points);
	// }

	for (size_t i = 0; i < outputTriangles.size(); i+=3) {

		triangles.push_back({outputTriangles[i].x, outputTriangles[i].y, 0.0f, 0x00000000});
		triangles.push_back({outputTriangles[i+1].x, outputTriangles[i+1].y, 0.0f, 0x00000000});
		triangles.push_back({outputTriangles[i+2].x, outputTriangles[i+2].y, 0.0f, 0x00000000});
		// std::vector<Vec2f> poly;
		// poly.push_back(Vec2f(outputTriangles[i].x,
		// 					 outputTriangles[i].y));
		// poly.push_back(Vec2f(outputTriangles[i+1].x,
		// 					 outputTriangles[i+1].y));
		// poly.push_back(Vec2f(outputTriangles[i+2].x,
		// 					 outputTriangles[i+2].y));
		// pumpkin::DebugManager::push_polygon(poly);

		indices.push_back(uint16_t(i));
		indices.push_back(uint16_t(i+1));
		indices.push_back(uint16_t(i+2));

	}

	vertex_count = outputTriangles.size();

	return vertex_count;
}

BgMask::BgMask(const std::vector<IndexPolygon>& polys) : m_polygons(polys) {	
}

void BgMask::setStencilIndex(int idx) {

	for (auto& layer : m_layers) {
		layer->m_render_passes[0].m_fstencil = 
			BGFX_STENCIL_TEST_NOTEQUAL
			| BGFX_STENCIL_FUNC_REF(idx)
			| BGFX_STENCIL_FUNC_RMASK(0xff)
			| BGFX_STENCIL_OP_FAIL_S_ZERO
			| BGFX_STENCIL_OP_FAIL_Z_REPLACE
			| BGFX_STENCIL_OP_PASS_Z_REPLACE;
	}

}
	
void BgMask::submit() {
	
	Eigen::Matrix4f t = Eigen::Matrix4f::Identity();
	float* mtx = t.data();
	
	// submit all layers to stencil buffer	
	for (auto& layer : m_layers)
	{
		bgfx::setIndexBuffer(layer->m_ibh);
		bgfx::setVertexBuffer(0, layer->m_vbh);

		for (auto pass : layer->m_render_passes) {		
			bgfx::setState(pass.m_state);			   		
			bgfx::setStencil(pass.m_fstencil, pass.m_bstencil);
			bgfx::submit(pass.m_viewId, pass.m_shader->getHandle());
		}
	}
}

void BgMask::craftStencilBuffer(const std::vector<IndexPolygon>& polys) {
	m_layers.clear();

	for(auto index_poly : polys)
	{		
		// one render pass per each depth
		int depth_index = index_poly.first;	   
		std::vector<PosColorVertex> vertices;
		std::vector<uint16_t> indices;
		auto poly = index_poly.second;
		makeTriangleStrip(poly.points, vertices, indices);
		StencilLayer::Ptr stencil_layer = StencilLayer::create(vertices, indices, depth_index);
		m_layers.push_back(stencil_layer);
	}
}
	
void BgMask::craftStencilBuffer(const Geometry::Polygon<Eigen::Vector2i>& mesh_poly) {
	
	m_layers.clear();

	for(auto index_poly : m_polygons)
	{		
		// one render pass per each depth
		int depth_index = index_poly.first;
		
		std::vector<PosColorVertex> vertices;
		std::vector<uint16_t> indices;

		auto poly = index_poly.second;
		Geometry::Polygon<Eigen::Vector2i> clipped_poly;
		
		if (Geometry::Clipper::clip<Eigen::Vector2i>(poly, mesh_poly, clipped_poly)) {
			makeTriangleStrip(clipped_poly.points, vertices, indices);
			StencilLayer::Ptr stencil_layer = StencilLayer::create(vertices, indices, depth_index);
			m_layers.push_back(stencil_layer);
		}
	}

	// reverse order!! TODO: make sure this works anywhere
//	std::reverse(m_layers.begin(), m_layers.end());
}

// different views/states for a mesh


// each mesh has 2 different render passes!!!
// 1. create stencil buffer (if necessary)
// 2. render mesh


void Mesh::submit(const float* mtx) {

	for (auto pass : m_render_passes) {
		bgfx::setTransform(mtx);
		bgfx::setIndexBuffer(m_dibh);
		bgfx::setVertexBuffer(0, m_dvbh);
		bgfx::setState(pass.m_state);			   		
		bgfx::setStencil(pass.m_fstencil, pass.m_bstencil);
		bgfx::submit(pass.m_viewId,
					 pass.m_shader->getHandle());
	}
}
	
}
