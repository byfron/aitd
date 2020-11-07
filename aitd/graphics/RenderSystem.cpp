#include "RenderSystem.hpp"
#include "AITDEngine.hpp"
#include "utils/Color.hpp"
#include <Components.hpp>

namespace aitd {

using namespace Components;

uint32_t getColor(int color_index, int poly_type) {

	ColorPalette::Ptr palette = ResourceManager::getResource<ColorPalette>(0);		
	Color c = palette->getColor(color_index);
	
	//TODO noise material
	// if (poly_type == 1) {
	// 	c.a = 254;
	// 	c.r = (unsigned char)((color_index % 16) * 16);
	// 	c.g = (unsigned char)((color_index / 16) * 16);
	// }
	// if (poly_type == 4 || poly_type == 5) {
	// 	c.a = 253;
	// 	c.r = 0;
	// 	c.g = 255;
	// 	c.b = (unsigned char)((color_index / 16) * 16);
	// }
	
	return Color::toABGR32(c);	
}

void MeshComponent::render(float delta, const float* mtx) {
		
	// TODO: this setUniform should be associated with a render pass! (now only acts on 0)
	mask_value[0] = static_cast<float>(depth_index);
	bgfx::setUniform(u_mask_value, &mask_value);
	
	mesh->submit(mtx);
}
	
// TODO: Render systems is super simple.
// Long time goal is to add lots of features (look at pumpkin)

void MeshComponent::generateMesh() {

	std::vector<PosColorVertex> vertex_buffer;
	std::vector<uint16_t> indices;
	int verticesCount = 0;
		
	for (auto prim : primitives) {

		verticesCount = vertex_buffer.size();
			
		if (prim->type == PRIM_TYPE_POLYGON) {
			auto poly_ptr = std::dynamic_pointer_cast<Polygon>(prim);
				
			uint32_t color = getColor(poly_ptr->color, poly_ptr->poly_type);				
				
			for (auto p : poly_ptr->points) {
				PosColorVertex pcv;
				pcv.m_x = vertices[p](0);
				pcv.m_y = vertices[p](1);
				pcv.m_z = vertices[p](2);										
				pcv.m_abgr = color;
				vertex_buffer.push_back(pcv);
			}

			//triangulate polygon
			int v0 = 0;
			int v1 = 1;
			int v2 = poly_ptr->points.size() - 1;
			bool swap = true;
			while (v1 < v2)
			{
				indices.push_back(verticesCount + v0);
				indices.push_back(verticesCount + v1);
				indices.push_back(verticesCount + v2);
				if (swap)
				{
					v0 = v1;
					v1++;
				}
				else
				{
					v0 = v2;
					v2--;
				}
				swap = !swap;
			}				
		}
	}
	
	mesh = std::make_shared<Mesh>();
	const bgfx::Memory* mem;
	mem = bgfx::copy(&vertex_buffer[0], sizeof(PosColorVertex) * vertex_buffer.size() );
	mesh->m_dvbh = bgfx::createDynamicVertexBuffer(mem, PosColorVertex::ms_decl);
		
	mem = bgfx::copy(&indices[0], sizeof(uint16_t) * indices.size() );
	mesh->m_dibh = bgfx::createDynamicIndexBuffer(mem);

	// TODO: move this to Mesh.cpp
	// By default we have two states: one to create the mesh masks, and another to render the mesh
	mesh->m_render_passes = {
		{

			/////////////////
			//
			// 1. in the pre_bgmask phase,  the stencil gets the mesh index
			//
			//////////////////
			
			// shader
			Shader::Ptr(new Shader("vs_default", "fs_default")),
			0
			//| BGFX_STATE_RGB_WRITE
			//| BGFX_STATE_ALPHA_WRITE
			//| BGFX_STATE_DEPTH_WRITE
			//| BGFX_STATE_DEPTH_TEST_LESS
			| BGFX_STATE_MSAA,
				// fstencil
			BGFX_STENCIL_TEST_ALWAYS
			| BGFX_STENCIL_FUNC_REF(depth_index)
			| BGFX_STENCIL_FUNC_RMASK(0xff)
			| BGFX_STENCIL_OP_FAIL_S_KEEP
			| BGFX_STENCIL_OP_FAIL_Z_KEEP
			| BGFX_STENCIL_OP_PASS_Z_KEEP,
			BGFX_STENCIL_NONE,
			RENDER_PASS_PRE_BGMASK
		},
		{
			// shader
			Shader::Ptr(new Shader("vs_default", "fs_default")),
			// render state
			0
			| BGFX_STATE_WRITE_RGB
			| BGFX_STATE_WRITE_A
			| BGFX_STATE_WRITE_Z
			| BGFX_STATE_DEPTH_TEST_LESS
			| BGFX_STATE_MSAA,
			// fstencil
			BGFX_STENCIL_TEST_EQUAL
			| BGFX_STENCIL_FUNC_REF(depth_index)
			| BGFX_STENCIL_FUNC_RMASK(0xff)
			| BGFX_STENCIL_OP_FAIL_S_KEEP
			| BGFX_STENCIL_OP_FAIL_Z_KEEP
			| BGFX_STENCIL_OP_PASS_Z_KEEP,
			// bstencil
			BGFX_STENCIL_NONE,
			// render view
			RENDER_PASS_GEOMETRY
		}
		};

	for (auto pass : mesh->m_render_passes) {
		pass.m_shader->init();
	}
	
	updateVertices();
}

void MeshComponent::updateVertices() {

	// Make sure rotations are up-to-date. TODO: This can be optimized
	for (auto& bone_it : skeleton->bone_map) {
		Bone::Ptr bone = bone_it.second;
		int parent_index = bone->parent_index;
		Eigen::Quaternionf R = bone->local_rotation;
		
		while(parent_index != -1) {
			Bone::Ptr parent = skeleton->bone_map[parent_index];
			R = R * parent->local_rotation;
			parent_index = parent->parent_index;
		}
		bone->global_rotation = R;
	}

	// modify vertex pool
	for (auto& bone_it : skeleton->bone_map) {
		Bone::Ptr bone = bone_it.second;
		int index = bone_it.second->start_vertex_index;
		for (int i = 0; i < bone_it.second->num_vertices_affected; i++) {
			vertices[index] = bone->global_rotation * offsets[index] + vertices[bone->local_pos_index];
			index++;
		}
	}	

	//TODO: Refactor this within mesh->updateVertices?
	std::vector<PosColorVertex> vertex_buffer;
	for (auto prim : primitives) {
		if (prim->type == PRIM_TYPE_POLYGON) {
			auto poly_ptr = std::dynamic_pointer_cast<Polygon>(prim);				
			uint32_t color = getColor(poly_ptr->color, poly_ptr->poly_type);				
			for (auto p : poly_ptr->points) {
				PosColorVertex pcv;
				pcv.m_x = vertices[p](0);
				pcv.m_y = vertices[p](1);
				pcv.m_z = vertices[p](2);										
				pcv.m_abgr = color;
				vertex_buffer.push_back(pcv);
			}
		}
	}
   
	const bgfx::Memory* mem;
	mem = bgfx::copy(&vertex_buffer[0], sizeof(PosColorVertex) * vertex_buffer.size() );
	bgfx::update(mesh->m_dvbh, 0, mem);
	///////////// 
}


void RenderSystem::update(EntityManager & em, EventManager &evm, float delta) {

	world->render(delta);

	// Update model mesh with current animation
	em.each<MeshComponent, AnimationComponent>(
		[delta](Entity entity,
				MeshComponent &mc,
				AnimationComponent &ac) {
			
			// perform interpolation
			ac.elapsed_time += delta;
			Skeleton::Ptr skel = ac.getInterpolatedSkeleton();
				
			// update mesh bones
			mc.updateSkeleton(skel);
		});

// go through all meshes in inverse dist to camera.
	// For each craft the corresponding stencil buffer with the "mesh index"
	// then the stencil test would be a less than
	struct MeshTrans {
		Entity entity;
		MeshComponent* mesh_comp;
		TransformComponent* tran_comp;
		float dist;
		bool operator<(const MeshTrans& a) {
			return dist > a.dist; //large distance first
		}		
	};

	auto cc = em.getComponentPtr<CameraComponent>(world->getCurrentCameraId());
	std::vector<MeshTrans> mesh_vector; 
	em.each<MeshComponent, TransformComponent>([&em, &cc, &mesh_vector, this](Entity mesh_entity,
																			  MeshComponent& mc,
																			  TransformComponent& tc) {
		   float dist = (cc->cam_pos - tc.getPosition()).norm();
		   mesh_vector.push_back({mesh_entity, &mc, &tc, dist});			   
    });
	std::sort(mesh_vector.begin(), mesh_vector.end());
	
	
	auto bzc = em.getComponentPtr<CameraBgZoneComponent>(world->getCurrentCameraId());
	bzc->resetStencilIndices();
	
	int depth_index = 1;

	// in PRE-BG pass, render all meshes to a framebuffer with the index of the mesh,
	// from most far to closest to the camera

	// in BG pass, render the stencil only in the regions that overlap with the corresponding
	// mesh mask
	
	

	// std::vector<BgMask::IndexPolygon> overlay_list;
	for (int i = 0; i < mesh_vector.size(); i++) {
		auto& mt = mesh_vector[i];

		Eigen::Matrix4f VP = cc->getVPMatrix();
		Eigen::Matrix4f MVP = VP * mt.tran_comp->getTransform();
//		Geometry::Polygon<Eigen::Vector2i>  mesh_mask = mt.mesh_comp->getProjectedMask(MVP);
	   
		auto ac = em.getComponentPtr<ActorCollisionComponent>(mt.entity.id());

		Eigen::Vector2f location(float(mt.tran_comp->getPosition()(0)),
								 float(mt.tran_comp->getPosition()(2)));
		
		// check in which zone the character is and update
		// the corresponding mask stencil state
		int mask_id = 0;
		for (auto& omz : bzc->overlay_masks_zones) {			
			bool isWithinZone = false;
			for (auto zone : omz.overlay_zones) {
				if (zone.isWithin(location)) {
					isWithinZone = true;
				}
			}												
			
			if (isWithinZone) {
				
				//TODO: simple test just check if BBoxes intersect and otherwise ignore the layer
				//intersect mask with mesh polygon
//					bzc->mask_vector[mask_id]->craftStencilBuffer(mesh_mask);
				
				// change corresponding mask status
				bzc->mask_vector[mask_id]->setStencilIndex(depth_index);
				bzc->mask_vector[mask_id]->submit();
			}


			
			mask_id++;

		}

		mt.mesh_comp->setDepthIndex(depth_index);
		depth_index++;
	}
	
	
	// for (auto mask : bzc->mask_vector) {
	// 	mask->submit();
	// }
		
	// Render all meshes
	em.each<MeshComponent, TransformComponent>(
		[delta](Entity entity,
				MeshComponent &mc,
				TransformComponent &tc) {
			//get transform
			mc.render(delta, tc.getTransform().data());
		});

	if (GraphicsEngine::show_debug_shapes) {
		// Render actor collision components
		em.each<ActorCollisionComponent>(
			[](Entity entity,
			   ActorCollisionComponent &ac) {
				Geometry::BBox tb = ac.bounding_box.getTransformedBox();
				DebugManager::push_aabb(tb.p_min,
										tb.p_max);


				std::vector<Vec3f> points;
				auto poly = ac.bounding_box.getBasePolygon();
				for (auto p : poly.points) {
					points.push_back(Vec3f(p(0), 0, p(1)));
				}
				DebugManager::push_polygon(points);			
		});

		em.each<TriggerComponent>(
			[](Entity entity,
			   TriggerComponent &trc) {
				Geometry::BBox tb = trc.bounding_box.getTransformedBox();
				DebugManager::push_aabb(tb.p_min,
										tb.p_max);
		});
		
		em.each<SceneCollisionComponent>(
			[](Entity entity,
			   SceneCollisionComponent &sc) {
				Geometry::BBox tb = sc.bounding_box.getTransformedBox();
				DebugManager::push_aabb(tb.p_min,
										tb.p_max);
		});


		// Render actor position
		auto tc_ptr = em.getComponentPtr<TransformComponent>(AITDEngine::player_entity_id);
		DebugManager::push_sphere(Sphere{bx::Vec3{tc_ptr->getPosition()(0), tc_ptr->getPosition()(1), tc_ptr->getPosition()(2)}, 40.0});

		//1014: stool
		//1011: window
		//1013: monster?
		//1016: lamp
		//1006: box
		tc_ptr = em.getComponentPtr<TransformComponent>(Entity::Id(1006));
		DebugManager::push_sphere(Sphere{bx::Vec3{tc_ptr->getPosition()(0), tc_ptr->getPosition()(1), tc_ptr->getPosition()(2)}, 440.0});

	// Render Camera Zones
	// em.each<CameraZoneComponent>(
	// 	[](Entity entity,
	// 	   CameraZoneComponent &czc) {
	// 		for (auto poly : czc.zones) {
	// 			std::vector<Vec3f> points;
	// 			for (auto p : poly.points) {
	// 				points.push_back(Vec3f(p(0), 0, p(1)));
	// 			}
	// 		 	DebugManager::push_polygon(points);
	// 		}			
	// 	});


	// Render camera bg zones
	em.each<CameraBgZoneComponent>(
		[delta](Entity entity,
				CameraBgZoneComponent& bzc) {
			for (auto omz : bzc.overlay_masks_zones) {
				for (auto zone : omz.overlay_zones) {				
					DebugManager::push_aabb(Vec3f(zone.min(0), 0.0, zone.min(1)),
											Vec3f(zone.max(0), 0.0, zone.max(1)));
				}
				break;
			}
		});
	}
	
	// // Render debug elements
	// em.each<DebugComponent>(
	// 	[delta](Entity entity,
	// 			DebugComponent &dc) {
	// 		dc.render(delta);
	// 	});
}
}
