#pragma once

#include "Actor.hpp" // remove this
#include <graphics/Mesh.hpp>
#include "RoomCamera.hpp"
#include <iostream>

namespace aitd {
namespace Components {

class TriggerComponent {
public:
	TriggerComponent(const Geometry::BBox& bb, const int16 t, const int16 param) :
		bounding_box(bb),
		type(t),
		parameter(param) {}
	
	bool isTriggering(const Eigen::Vector3f& pos) {
		auto poly = bounding_box.getBasePolygon();
		return poly.isWithin(Vec2f(pos(0), pos(2)));
	}
	
	Geometry::BBox bounding_box;
	int16 parameter; 
	int16 type; //0: room change
};
	
class SceneCollisionComponent {
public:

	SceneCollisionComponent(const Geometry::BBox& bb) : bounding_box(bb) {}

	bool checkCollision(SceneCollisionComponent* box) const {
		uint32_t int_code = bounding_box.getTransformedBox().intersect(
			box->bounding_box.getTransformedBox());
		return int_code == 0;
	}

	void reset() {
		is_colliding_with_actor = false;
	}
	
	bool is_colliding_with_actor = false;
	Entity::Id colliding_with_entity;
	
	Geometry::BBox bounding_box;
};
	
class ActorCollisionComponent : public SceneCollisionComponent {
public:

	ActorCollisionComponent(const Geometry::BBox& bb) : SceneCollisionComponent(bb) {}

	int getCollidingActorIdx() { return 0; }
	int getCollidingWithSceneIdx() { return 0; }

	
	int getHittingActorIdx() { return 0; }	 // TODO: this may go to another component	

	void reset() {
		SceneCollisionComponent::reset();
		is_colliding_with_scene = false;
	}

	bool is_colliding_with_scene = false;
	Entity::Id colliding_with_entity;
};
	
class ScriptComponent {
public:
	ScriptComponent(int id) : script_id(id) {
	}   

	int script_id = 0;
	
protected:

};
	
class UserInputComponent {
public:
};

//This is a temporary component until we figure out each data field
//so that we can store it properly in components (we could also have a global table)
class MetaDataComponent {
public:

	MetaDataComponent(int16 index, int16 f, int16 body, int16 l, int16 lm, int16 tm) :
		object_id(index),
		flags(f),
		body_num(body),
		life(l),
		life_mode(lm),
		track_mode(tm) {
			
	}

	int16 object_id; // corresponding object/body in the table of bodies pre-loaded
	int16 flags; //
	int16 body_num;
	int16 life;
	int16 life_mode;
	int16 track_mode;



	//added to make it compile. review
	int16 anim;
	int16 field44;
	
};	
	
class AnimationComponent {
public:   	
	AnimationComponent(int id, Animation::Ptr a) : anim_id(id), anim(a) {
		//TODO: we should instead deep copy the data here otherwise we break our cache-friendly design

		current_skeleton = Skeleton::Ptr(new Skeleton());
	}

	Skeleton::Ptr getInterpolatedSkeleton() {

		//length is in frames (we should divide by the framerate?)
		alpha = elapsed_time/(float(anim->keyframes[current_frame]->length)/30);

		if (alpha > 1.0f) {
			alpha = 0.0f;		   
			elapsed_time = 0.0f;
			current_frame +=1;
			next_frame += 1;
			if (next_frame == anim->keyframes.size()) {
				next_frame = 0;
			}
			if (current_frame == anim->keyframes.size()) {
				current_frame = 0;
			}
		}
		
		return interpolateSkeletons(anim->keyframes[current_frame]->skeleton,
									anim->keyframes[next_frame]->skeleton,
									alpha);
	}

	// int anim_type;
	float alpha = 0; //interpolation factor
	int duration = 0;
	float elapsed_time = 0;
	
	int current_frame = 0;
	int next_frame = 1;
	
	// std::vector<Frame> frames;

	Skeleton::Ptr current_skeleton;   
	
	int anim_id = -1; //animation id (related to the game logic)

	// equivalent of fields in actor. Maybe we don't need this at all.
	int end_frame; // frame needs to change
	int end_anim; // end of animation
	
	Animation::Ptr anim;

};

// Encodes masking/stencil of background image areas
// TODO: move this to /graphics	
class CameraBgZoneComponent {
public:
	CameraBgZoneComponent(const std::vector<CameraBackgroundLayer::OverlayMaskZone>& omz) :
		overlay_masks_zones(omz) {

		std::reverse(overlay_masks_zones.begin(), overlay_masks_zones.end());
		
		for (auto omz : overlay_masks_zones)
		{
			std::vector<BgMask::IndexPolygon> overlay_list;
			for (auto overlay : omz.overlays) {
			 	overlay_list.push_back(BgMask::IndexPolygon(omz.depth_index, overlay));
			}
			BgMask::Ptr mask = std::make_shared<BgMask>(overlay_list);
			mask->craftStencilBuffer(overlay_list);
			mask_vector.push_back(mask);
		}
	}

	void resetStencilIndices() {
		for (auto mask : mask_vector) {
			mask->setStencilIndex(0);
		}
	}
	
	std::vector<CameraBackgroundLayer::OverlayMaskZone> overlay_masks_zones; //TODO: dont need to store this

	// craft a mask per each mask zone!	
	std::vector<BgMask::Ptr> mask_vector;
	
};

class CameraZoneComponent {
public:
	CameraZoneComponent(std::vector<Geometry::Polygon< Vec2f > >& z) : zones(z) {}
	std::vector<Geometry::Polygon<Vec2f> > zones;
};
	
class MoveComponent {
public:
	MoveComponent(float ms, float rs) : move_speed(ms),
										rotate_speed(rs),
										move_vector(-Vec3f::UnitZ()) {}
	float move_speed = 0;
	float rotate_speed = 0;
	Vec3f move_vector; //goto_vector

	void reset() {
		translation = Eigen::Vector3f::Zero();
		rotation = Eigen::Matrix3f::Identity();
	}
	
	Eigen::Vector3f translation = Eigen::Vector3f::Zero();
	Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
};


class TransformComponent {
public:
	TransformComponent(int16 x, int16 y, int16 z, const Eigen::Matrix3f& rot) :
		translation(Eigen::Vector3f(x, y, z)),
		rotation(rot) {
	}

	Eigen::Vector3f getPosition() {
		return (getTransform().col(3).head(3));// * Eigen::Matrix<float, 4, 1>(0.0, 0.0, 0.0, 1.0)).head(3) ;
	}
	
	Eigen::Matrix4f getTransform() {
		Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
		T.col(3).head(3) = translation;
		T.topLeftCorner(3,3) = rotation;
		return T;
	}

	Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
	Eigen::Vector3f translation = Eigen::Vector3f::Zero();
};
	
};
}
