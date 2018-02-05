#include "RoomCamera.hpp"
#include <common/endian.h>
#include <utils/Color.hpp>
#include <utils/DataParsing.hpp>
#include <utils/Geometry.hpp>
//TODO: remove OpenCV when done debugging
#include <opencv2/opencv.hpp>

namespace aitd {
typedef Eigen::Matrix<int,2,1> Vector2i;

CameraZoneEntry::CameraZoneEntry(const char* data) {
	load(data);
}

void CameraZoneEntry::load(const char* data) {

	uint16 num_of_points = READ_LE_UINT16(data);
	data += 2;

	std::vector<Eigen::Vector2i> unsorted_points;
	Vector2i center = Vector2i::Zero();
	for(uint16 point_idx = 0; point_idx < num_of_points; point_idx++) {
		Vector2i point;
		point(0) = (int16)READ_LE_UINT16(data);
		data += 2;
		point(1) = (int16)READ_LE_UINT16(data);
		data += 2;		
		unsorted_points.push_back(point * 10);
		center += point;
	}
	      
	//order points by angle
	assert(num_of_points > 0);
	center /= num_of_points;
	struct Pair {
		float angle;
		int idx;
		bool operator<(const Pair& a) {
			return angle > a.angle;
		}
	};
	
	std::vector<Pair> angle_vector(num_of_points);	
	for(uint16 point_idx = 0; point_idx < num_of_points; point_idx++) {
		Vector2i v1 = unsorted_points[point_idx] - center;
		float angle = atan2(v1(0), v1(1));
		angle_vector[point_idx] = Pair{angle, point_idx};
	}
	std::sort(angle_vector.begin(), angle_vector.end());
	
	points.clear();
	for (auto p : angle_vector) {
		points.push_back(unsorted_points[p.idx]);
	}
 	points.push_back(points[0]);

}

CameraZone::CameraZone(const char* data, const char* base_data) {
	load(data, base_data);
}

void CameraZone::load(const char* data, const char* base_data) {

	//TODO: rename these puppies
	dummy1 = READ_LE_UINT16(data + 0x00); //room num. 
	dummy2 = READ_LE_UINT16(data + 0x02); //offset from base_data to the bg overlay
	dummy3 = READ_LE_UINT16(data + 0x04); //offset from base_data to the zoneEntry data
	dummy4 = READ_LE_UINT16(data + 0x06);
	dummy5 = READ_LE_UINT16(data + 0x08);
	dummy6 = READ_LE_UINT16(data + 0x0A);

	const char *p_zone_data = base_data + dummy3;

	uint16 num_zones = READ_LE_UINT16(p_zone_data);
	p_zone_data += 2;
	
	for(int j = 0; j < num_zones; j++) {
		CameraZoneEntry::Ptr czentry =
			CameraZoneEntry::Ptr(new CameraZoneEntry(p_zone_data));
		p_zone_data += 2 + (4 * (czentry->points.size()-1));
		entry_vector.push_back(czentry);
	}
}

CameraBackgroundLayer::CameraBackgroundLayer(const char* data) {
	load(data);
}

void CameraBackgroundLayer::load(const char* data) {
	int num_overlay_zones = *(int16 *)(data);

	const char *curr_data = data + 2;

	std::cout << "num zones:" << num_overlay_zones << std::endl;
	
	for (int i = 0; i < num_overlay_zones; i++) {

		OverlayMaskZone omz;
		
		int num_params = *(uint16 *)(curr_data);		
		const char *src = data + *(uint16 *)(curr_data + 2);

		// This zone is used to determine if the overlay affects the actor
		// For instance a monster behind a wall should be affected by the wall overlays,
		// but not the main character walking within a camera room
		// We will need multiple stencil buffers? or to sort out the rendering order?

		// When rendering each actor if it's in a zone affected, build a stencil buffer
		// and apply it 

		const char* param_data = curr_data + 4;
		for (int j = 0; j < num_params; j++) {
			Geometry::Quad overlay_zone;
			int16 zoneX1 = *(int16 *)(param_data);
			int16 zoneZ1 = *(int16 *)(param_data + 2);
			int16 zoneX2 = *(int16 *)(param_data + 4);
			int16 zoneZ2 = *(int16 *)(param_data + 6);
			overlay_zone.min = Vec2f(zoneX1, zoneZ1)*10;
			overlay_zone.max = Vec2f(zoneX2, zoneZ2)*10;
			omz.overlay_zones.push_back(overlay_zone);
			param_data += 0x8;

			
		} 

		//const char *src_tmp = src;
		int num_overlays = *(int16 *)src;
		
		src += 2;

		for (int j = 0; j < num_overlays; j++) {
			int size = *(int16 *)(src);
			src += 2;
			Geometry::Polygon<Vec2i> overlay = createOverlay((int16*)src, size);
			omz.overlays.push_back(overlay);
			src += size * 4;
		}


		overlay_masks_zones.push_back(omz);

		curr_data += 2;
		curr_data += ((num_params * 4) + 1) * 2;

		//curr_data = src_tmp + 2 + ((num_overlays * 4) + 1)*2;
	}

}

void CameraBackgroundLayer::sortZonesFromCamera(const Vec3f& camera_pos) {


	int di = 1;
	for (auto& omz : overlay_masks_zones) {
		omz.depth_index = di;
		di++;
	}
	return;

	
	struct Pair {
		float dist;
		int idx;
		// bool operator<(const Pair& a) {
		// 	return dist < a.dist;
		// }
	};
	
	// sort zones acoording to the distance to the camera and assing them a "depth order"
	std::vector<Pair> dist_vector;
	int idx = 0;
	for (auto omz : overlay_masks_zones) {
		float max_dist = 0;		
		for (auto zone : omz.overlay_zones) {


			// compute distance point quad
			float d = zone.distanceFrom(camera_pos);
			
			// Vec3f min_pos = Vec3f(zone.min(0), 0, zone.min(1));
			// Vec3f max_pos = Vec3f(zone.max(0), 0, zone.max(1));
			// float d = std::max((camera_pos - min_pos).norm(),
			// 				   (camera_pos - max_pos).norm());
			if (d > max_dist) {
				max_dist = d;
			}
		}

		dist_vector.push_back(Pair{max_dist, idx});
		idx++;
	}

	// sort dist vector from min to max dist
//	std::sort(dist_vector.begin(), dist_vector.end());
//	std::reverse(dist_vector.begin(), dist_vector.end());
 
	// assign depth indices to zones
	int depth_index = 1; //zero is the default value
	int ii = 0;
	
	for (auto pair : dist_vector) {
		overlay_masks_zones[ii].depth_index = depth_index;
		overlay_masks_zones[ii].dist = pair.dist;		
		depth_index++;
		ii++;
	}
}

Geometry::Polygon<Vec2i> CameraBackgroundLayer::createOverlay(const int16* base_data, int size) {

	int min1 = 32767, min2 = 32767;
	int max1 = -32768, max2 = -32768;
	const int16* data = base_data;
	Geometry::Polygon<Vec2i> overlay;


	// compute scale of the overlay
	// TODO: get from engine instance!? scale a posteriori!!
	float scale_x = float(1280)/320;
	float scale_y = float(720)/200;
	
	for(int i = 0; i < size; i++) {
		
		int tmp1 = data[0];
		int tmp2 = data[1];

		if (tmp1 < min1) {
			min1 = tmp1;
		}
		if (tmp1 > max1) {
			max1 = tmp1;
		}
		if (tmp2 < min2) {
			min2 = tmp2;
		}
		if (tmp2 > max2) {
			max2 = tmp2;
		}
		
		overlay.points.push_back(Vec2i(tmp1 * scale_x, tmp2 * scale_y));
		
		data += 2;
	}

	return overlay;
	
	// data = base_data;
	// for(int i = 0; i < size; i++) {
		
	// 	int px = data[0];
	// 	int py = data[1];

	// }
	
}

RoomCamera::RoomCamera(const char* data) {//, int index) {
	load(data);//, index);
}

RoomCamera::~RoomCamera() {
	delete [] background_image;
}

void RoomCamera::load(const char *base_data) {//, int index) {

//	uint32 offset = READ_LE_UINT32(base_data + index * 4);
	const char* data = base_data;// + offset;
	
 	uint16 alpha = READ_LE_UINT16(data + 0x00);
	uint16 beta  = READ_LE_UINT16(data + 0x02);
	uint16 gamma = READ_LE_UINT16(data + 0x04);
	
	int16 x = READ_LE_UINT16(data + 0x06); // (x - world_x) * 10
	int16 y = -READ_LE_UINT16(data + 0x08); // -(world_y - y) * 10
	int16 z = READ_LE_UINT16(data + 0x0A); //(world_z - z) * 10
	
	position = Vec3f(float(x), float(y), float(z))*10;
	
	focal1 = READ_LE_UINT16(data + 0x0C);
	focal2 = READ_LE_UINT16(data + 0x0E);
	focal3 = READ_LE_UINT16(data + 0x10);

	// Build projection matrix!
	float fx = focal2;
	float fy = focal3;
	float fz = focal1; // no idea where this param comes from or what is for (*)
	float s = 0;
	float zmin = 0;
	float zmax = 1000000;	
	float W = GraphicsEngine::WIDTH;
	float H = GraphicsEngine::HEIGHT;
	float cx = GraphicsEngine::WIDTH/2;
	float cy = GraphicsEngine::HEIGHT/2;
	
	projection = Eigen::Matrix4f::Zero();
	projection(0,0) = 2*fx/W;
	projection(0,1) = 2*s/W;
	projection(1,1) = 2*fy/H;
	projection(0,2) = 2*(cx/W)-1;
	projection(1,2) = 2*(cy/H)-1;
	projection(2,2) = (zmax+zmin)/(zmax-zmin);
	projection(3,2) = 1;
	projection(2,3) = 2*zmax*zmin/(zmin-zmax);
	projection(3,3) = fz; // (*) but has to go here

	// Build view matrix
	float cosx = DataParsing::computeCos(alpha&0x3FF);
	float cosy = DataParsing::computeCos(beta&0x3FF);
	float cosz = DataParsing::computeCos(gamma&0x3FF);
	float sinx = DataParsing::computeSin(alpha&0x3FF);
	float siny = DataParsing::computeSin(beta&0x3FF);
	float sinz = DataParsing::computeSin(gamma&0x3FF);
	
	transform = Eigen::Matrix4f::Identity();

	Eigen::Matrix3f rotX = Geometry::getXRotMat(sinx, cosx);
	Eigen::Matrix3f rotY = Geometry::getYRotMat(siny, -cosy);
	Eigen::Matrix3f rotZ = Geometry::getZRotMat(sinz, cosz);
		
	if (!beta) {
		rotY.col(0) = Vector3f::UnitX();
		rotY.col(2) = Vector3f::UnitZ();
		rotY = Eigen::Matrix3f::Identity();
	}
	if (!alpha) {
		rotX.col(1) = Vector3f::UnitY();
		rotX.col(2) = Vector3f::UnitZ();
		rotX = Eigen::Matrix3f::Identity();
	}
	if (!gamma) {
		rotZ.col(0) = Vector3f::UnitX();
		rotZ.col(1) = Vector3f::UnitY();
		rotZ = Eigen::Matrix3f::Identity();
	}

	Eigen::Matrix3f t = Eigen::Matrix3f(rotZ * rotY * rotX);	
	transform.topLeftCorner(3,3) = t;
	transform.col(3).head(3) = position;
	
	int16 num_camera_zone_def = READ_LE_UINT16(data + 0x12);
	const char* base_zone_data = data;
	data += 0x14;

	// This is one camera zone per room!!?
	for(int k = 0; k < num_camera_zone_def; k++) {
		CameraZone::Ptr zone = CameraZone::Ptr(new CameraZone(data, base_zone_data));
		zone_vector.push_back(zone);

		CameraBackgroundLayer::Ptr bglayer =
			CameraBackgroundLayer::Ptr(new CameraBackgroundLayer(base_zone_data + zone->dummy2));

		bglayer->sortZonesFromCamera(position);
		
		bglayer_vector.push_back(bglayer);
		data += 0x0C;
	}
}

void RoomCamera::loadBackgroundImage(const char* data) {
	ColorPalette::Ptr color_palette =
		ResourceManager::getResource<ColorPalette>(OBJECT_PALETTE_ID);
	background_image = new unsigned char[320*200*3];

	cv::Mat image = cv::Mat(200, 320, CV_8UC3);
	
	for (int i = 0; i < 320; i++) {
		for (int j = 0; j < 200; j++) {

			const unsigned char color_index = data[i*200 + j];
			Color c = color_palette->getColor(color_index);

			background_image[(i*200 + j)*3 + 0] = c.r;
			background_image[(i*200 + j)*3 + 1] = c.g;
			background_image[(i*200 + j)*3 + 2] = c.b;

			//image.at<cv::Vec3b>(i*200 + j) = cv::Vec3b(c.b, c.g, c.r);
		}
	}

// 	int r = 3;
// 	for (auto layer : bglayer_vector) {
// //		for (auto over : layer->overlays)
// 		{
// 			auto over = layer->overlays[6];
// 			for (auto p : over.points) {
// 				int x = p(0);
// 				int y = p(1);
// 				cv::circle(image, cv::Point2d(x, y), r, cv::Scalar(255, 0, 0));
// 			}
// 			break;
// 		}
// 	}

// 	cv::resize(image, image, cv::Size(320*4, 200*4));
// 	cv::imshow("im" , image);
// 	cv::waitKey(0);


}
}
