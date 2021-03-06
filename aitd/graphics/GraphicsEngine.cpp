#include "GraphicsEngine.hpp"
#include "ResourceManager.hpp"
#include "DebugManager.hpp"
#include "GameSystems.hpp"
#include <iostream>
#include <common/common.h>

namespace aitd {

Camera GraphicsEngine::m_camera;
bool GraphicsEngine::m_debug = false;
int GraphicsEngine::WIDTH = 320;
int GraphicsEngine::HEIGHT = 200;
bool GraphicsEngine::show_debug_shapes = true;
	
std::vector<Aabb> DebugManager::aabb_vec;
std::vector<Cylinder> DebugManager::cyl_vec;
std::vector<Sphere> DebugManager::sphere_vec;
std::vector<std::vector<Vec3f>> DebugManager::poly_vec;
std::vector<DebugManager::Line> DebugManager::line_vec;
DebugDrawEncoder DebugManager::dde;

bgfx::VertexLayout PosColorVertex::ms_decl;
bgfx::VertexLayout PosTexCoordVertex::ms_decl;

//TODO: put in utils
void screenSpaceQuad(float _textureWidth, float _textureHeight, bool _originBottomLeft = false, float _width = 1.0f, float _height = 1.0f)
{
	if (3 == bgfx::getAvailTransientVertexBuffer(3, PosTexCoordVertex::ms_decl) )
	{
		bgfx::TransientVertexBuffer vb;
		bgfx::allocTransientVertexBuffer(&vb, 3, PosTexCoordVertex::ms_decl);
		PosTexCoordVertex* vertex = (PosTexCoordVertex*)vb.data;

		const float zz = 0.0f;

		const float minx = -_width;
		const float maxx =  _width;
		const float miny = 0.0f;
		const float maxy = _height*2.0f;

		const bgfx::Caps* caps = bgfx::getCaps();
		const float s_texelHalf = bgfx::RendererType::Direct3D9 == caps->rendererType ? 0.5f : 0.0f;
		const float texelHalfW = s_texelHalf/_textureWidth;
		const float texelHalfH = s_texelHalf/_textureHeight;
		const float minu = -1.0f + texelHalfW;
		const float maxu =  1.0f + texelHalfW;

		float minv = texelHalfH;
		float maxv = 2.0f + texelHalfH;

		if (_originBottomLeft)
		{
			float temp = minv;
			minv = maxv;
			maxv = temp;

			minv -= 1.0f;
			maxv -= 1.0f;
		}

		vertex[0].m_x = minx;
		vertex[0].m_y = miny;
		vertex[0].m_z = zz;
		vertex[0].m_u = minu;
		vertex[0].m_v = minv;

		vertex[1].m_x = maxx;
		vertex[1].m_y = miny;
		vertex[1].m_z = zz;
		vertex[1].m_u = maxu;
		vertex[1].m_v = minv;

		vertex[2].m_x = maxx;
		vertex[2].m_y = maxy;
		vertex[2].m_z = zz;
		vertex[2].m_u = maxu;
		vertex[2].m_v = maxv;

		bgfx::setVertexBuffer(0, &vb);
	}
}


float getDeltaTime() {
	int64_t now = bx::getHPCounter();
	static int64_t last = now;
	const int64_t frameTime = now - last;
	last = now;
	const double freq = double(bx::getHPFrequency() );
	return float(frameTime/freq);
}

void GraphicsEngine::start(int _argc, const char* const* _argv) {
	Args args(_argc, _argv);


	bgfx::Init init;
	init.type     = args.m_type;
	init.vendorId = args.m_pciId;
	init.resolution.width  = 800;
	init.resolution.height = 600;
	bgfx::init(init);

	m_width = WIDTH;
	m_height = HEIGHT;

	bgfx::reset(m_width, m_height, InputManager::m_reset);
	bgfx::setDebug(InputManager::m_debug);

	// Set palette color for index 0
	bgfx::setPaletteColor(0, UINT32_C(0x000000000) );
	
	// Set palette color for index 1
	bgfx::setPaletteColor(1, UINT32_C(0x303030ff) );

	bgfx::setViewClear(RENDER_PASS_BACKGROUND
					   , BGFX_CLEAR_COLOR|BGFX_CLEAR_DEPTH|BGFX_CLEAR_STENCIL
					   , 0x30303000
					   , 1.0f
					   , 100);

	bgfx::setViewClear(RENDER_PASS_PRE_BGMASK
					   , BGFX_CLEAR_STENCIL|BGFX_CLEAR_DEPTH|BGFX_CLEAR_STENCIL
					   , 0x30303000
					   , 1.0f
	 				   , 100);
	
	bgfx::setViewClear(RENDER_PASS_BGMASK //TODO: rename STENCIL
					   , BGFX_CLEAR_STENCIL
					   , 0x30303000
					   , 1.0f
	 				   , 100);
	
	bgfx::setViewClear(RENDER_PASS_GEOMETRY
					   , BGFX_CLEAR_COLOR| BGFX_CLEAR_DEPTH|BGFX_CLEAR_STENCIL
					   , 0x30303000
					   , 1.0f
					   , 100);
	
	initResources();

	init_engine();
	ddInit();
	//ddSetColor(0xff00ff00);
	//ddSetWireframe(true);

	m_camera.init();
	m_input_manager.init();

}

void GraphicsEngine::initResources() {
	PosColorVertex::init();
	PosTexCoordVertex::init();

	u_preMaskTex  = bgfx::createUniform("u_preMaskTex",  bgfx::UniformType::Sampler);

}

void GraphicsEngine::run() {

	const float deltaTime = getDeltaTime();

	
	// bgfx::setViewClear(RENDER_PASS_BACKGROUND
	// 				   , BGFX_CLEAR_COLOR|BGFX_CLEAR_DEPTH|BGFX_CLEAR_STENCIL
	// 				   , 0x30303000
	// 				   , 1.0f
	// 				   , 1);

	bgfx::setViewClear(RENDER_PASS_PRE_BGMASK //TODO: rename STENCIL
					   , BGFX_CLEAR_STENCIL
					   , 0x30303000
					   , 1.0f
					   , 0);
	
	bgfx::setViewClear(RENDER_PASS_BGMASK //TODO: rename STENCIL
					   , BGFX_CLEAR_STENCIL
					   , 0x30303000
					   , 1.0f
					   , 0);
	
	bgfx::setViewClear(RENDER_PASS_GEOMETRY
					   , BGFX_CLEAR_DEPTH
					   , 0x30303000
					   , 1.0f
					   , 0);


//	DebugManager::push_polygon({Vec3f(0.f,0.f,0.f), Vec3f(1.f,1.f,1.f), Vec3f(0.f,2.f,1.f)});
	
	// If the size of the window changes, update the size of.framebuffers
	if (m_oldWidth  != m_width  ||  m_oldHeight != m_height) {
		m_oldWidth = m_width;
		m_oldHeight = m_height;

		const uint32_t samplerFlags = 0
			| BGFX_TEXTURE_RT
			| BGFX_SAMPLER_MIN_POINT
			| BGFX_SAMPLER_MAG_POINT
			| BGFX_SAMPLER_MIP_POINT
			| BGFX_SAMPLER_U_CLAMP
			| BGFX_SAMPLER_V_CLAMP;


		m_gbufferTex[0] = bgfx::createTexture2D(m_width, m_height, false, 1, bgfx::TextureFormat::BGRA8, samplerFlags);

		m_preMaskBuffer = bgfx::createFrameBuffer(BX_COUNTOF(m_gbufferTex), m_gbufferTex, true);

	}

/////////////////////////// All this has to go away ////////////////////
	
	// Set geometry view
	float view[16];
	float proj[16];

	bgfx::touch(0);
	
	// background render view //////////////////////////////
	bx::mtxOrtho(proj, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 100.0f, 0.0f, bgfx::getCaps()->homogeneousDepth);
	bgfx::setViewRect(RENDER_PASS_BACKGROUND, 0, 0, m_width, m_height);
	bgfx::setViewTransform(RENDER_PASS_BACKGROUND, NULL, proj);

	// pre-mask : mesh masks view //////////////////////////
	bgfx::setViewRect(RENDER_PASS_PRE_BGMASK, 0, 0, m_width, m_height);
//	bx::mtxOrtho(proj, 0.0f, m_width, m_height, 0.0f, 0.0f, 1000000.0f, 0.0f, bgfx::getCaps()->homogeneousDepth);
//	bgfx::setViewFrameBuffer(RENDER_PASS_PRE_BGMASK, m_preMaskBuffer);	
		
	// Stencil render view (frame buffer) /////////////////// 
	bgfx::setViewRect(RENDER_PASS_BGMASK, 0, 0, m_width, m_height);
	bx::mtxOrtho(proj, 0.0f, m_width, m_height, 0.0f, 0.0f, 1000000.0f, 0.0f, bgfx::getCaps()->homogeneousDepth);
	bgfx::setViewTransform(RENDER_PASS_BGMASK, NULL, proj);
	// TODO: we may have to move this before we submit the stencil
//	bgfx::setTexture(1, u_preMaskTex,  bgfx::getTexture(m_preMaskBuffer, 0) );

	// Geometry render view
	bgfx::setViewRect(RENDER_PASS_GEOMETRY, 0, 0, uint16_t(m_width), uint16_t(m_height));
//	bgfx::setViewTransform(RENDER_PASS_GEOMETRY, NULL, proj);
	
	DebugManager::update(deltaTime);
	frame(deltaTime);
	
	bgfx::frame();
}

void GraphicsEngine::stop() {
	ddShutdown();
	bgfx::shutdown();

}
}
