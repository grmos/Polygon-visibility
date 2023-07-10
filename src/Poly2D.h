#ifndef Poly2D_H
#define Poly2D_H

#include <VVRScene/scene.h>
#include <VVRScene/canvas.h>
#include <VVRScene/utils.h>
#include <GeoLib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <algorithm>
#include <math.h>

#define PI 3.14159265

class POLY_2D : public vvr::Scene
{
public:
	POLY_2D();
	struct lineEquation {
		double a;
		double b;
		double c;
		C2DLine line;
	};

	struct circleEquation
	{
		double a;
		double b;
		double c;
		C2DCircle circle;
	};
	lineEquation  createLineEquation(const C2DLine& line);
	lineEquation createverticalLineEquation(const C2DLine& line, const C2DPoint& point);
	void line_intersection_point(const lineEquation line1, const lineEquation line2, C2DPoint& intersection_pt);
	circleEquation createCircleEquation(const C2DCircle& circle);
	int min_dist(C2DPointSet *Set, C2DPoint p);
	bool lineContainsPoint(const lineEquation& line, const C2DPoint p);
	void intersection_in_polygon(const C2DPolygon* poly, C2DPointSet* Set);
	void task_2(const C2DPolygon* q);
	void find_sign(const C2DPolygon* poly, int array[]);
	void getPointIntersection(const circleEquation circle, const lineEquation line, C2DPoint& point1, C2DPoint& point2);
	bool IsPositive(const lineEquation line, C2DPoint p);
	void check_sign(const C2DPolygon* poly, int array[], C2DPointSet* Set);
	bool check_pointinline(lineEquation line, C2DPoint p);
	void ConvexHull_Fast();
	void tr_pointset_to_lineset(C2DLineSet* LSet, C2DPointSet* PSet,int position);
	int task_1(const C2DPoint* p1, const C2DPolygon* q);

protected:
    void draw() override;
    void reset() override;
	void mousePressed(int x, int y, int modif) override;
    void mouseReleased(int x, int y, int modif) override;
	void mouseMoved(int x, int y, int modif); //override;
	void keyEvent(unsigned char key, bool up, int modif) override;

private:
	void polygon_creation(C2DPointSet *set, C2DPolygon *poly);

private:
    vvr::Canvas2D my_camvas;
	vvr::Canvas2D my_camvas_task_2;
	vvr::Canvas2D my_camvas_3;
	C2DLineSet lineset;
	C2DPointSet my_point_cloud;
	C2DPolygon my_polygon;
	vvr::Point2D* my_mouse_pos;
	C2DPointSet my_polyset;
	C2DPolygon my_vision_polygon;
	C2DPointSet inter2_set;
	C2DPointSet convex_set;
	C2DPolygon core_poly;
	bool m_run_task_2 = false;;
	C2DLineSet po_lineset;
};
#endif 
