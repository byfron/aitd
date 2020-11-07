#pragma once

#include "math.hpp"
#include "graphics/GraphicsEngine.hpp"
#include <common/bgfx_utils.h>
#include <common/debugdraw/debugdraw.h>
#include <vector>

namespace aitd {
	
class DebugManager {

public:
	static const uint32_t default_color = 0xff00ff00;

	struct Line {
		Vec3f a;
		Vec3f b;
		uint32_t color;
	};
	
	template <typename T>
	static void draw(T & object) {
		dde.setWireframe(true);
		dde.draw(object);
	}

	static void drawLine(const Line& l) {
		dde.setColor(l.color);
		dde.moveTo(l.a(0), l.a(1), l.a(2));
		dde.lineTo(l.b(0), l.b(1), l.b(2));
		dde.setColor(0xff00ff00);
	}
	
	static void drawPoly(const std::vector<Vec3f> & poly) {
		Vec3f first_vert = poly[0];
		dde.moveTo(first_vert(0), first_vert(1), first_vert(2));
		for (int i = 0; i < poly.size(); i++) {
			dde.lineTo(poly[i](0), poly[i](1), poly[i](2));
		}
		dde.lineTo(first_vert(0), first_vert(1), first_vert(2));
	}

	static void update(float delta) {

		
		dde.begin(RENDER_PASS_GEOMETRY);		
		dde.push();
		dde.setColor(0xff00ff00);
		dde.setWireframe(true);

		for (auto aabb : aabb_vec) {
			//dde.draw(aabb);
		}

		for (auto cyl : cyl_vec) {
			//dde.drawCylinder(cyl.pos, cyl.end, cyl.radius);
		}

		for (auto poly : poly_vec) {
			//drawPoly(poly);
		}

		for (auto line : line_vec) {
			//drawLine(line);
		}

		for (auto sphere : sphere_vec) {
			//draw(sphere);
		}

		dde.pop();
		dde.end();

		clear();
	}

	static void push_cyl(Vec3f from, Vec3f to, float rad) {
		Cylinder c;
		FROM_V3(c.pos, from);
		FROM_V3(c.end, to);
		c.radius = rad;
		cyl_vec.push_back(c);
	}

	static void push_polygon(const std::vector<Vec3f> & poly) {
		poly_vec.push_back(poly);
	}	

	static void push_line(const Vec3f& a, const Vec3f& b, uint32_t color = default_color) {
		line_vec.push_back({a, b, color});
	}

	
	static void push_aabb(Vec3f a, Vec3f b) {
		Aabb box;
		box.min.x = a(0);
		box.min.y = a(1);
		box.min.z = a(2);
		box.max.x = b(0);
		box.max.y = b(1);
		box.max.z = b(2);
		aabb_vec.push_back(box);
	}

	static void push_sphere(const Sphere& s) {
		sphere_vec.push_back(s);
	}

	static void clear() {
		cyl_vec.clear();
		aabb_vec.clear();
		poly_vec.clear();
		line_vec.clear();
		sphere_vec.clear();
	}

protected:

	static std::vector<Aabb> aabb_vec;
	static std::vector<Cylinder> cyl_vec;
	static std::vector<Sphere> sphere_vec;
	static std::vector<Line> line_vec;
	static std::vector<std::vector<Vec3f>> poly_vec;
    static DebugDrawEncoder dde;

};

}