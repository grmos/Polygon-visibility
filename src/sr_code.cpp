#include "Poly2D.h"

#define MAX_NUM_PTS 12

using namespace std;
using namespace vvr;
 
int main(int argc, char* argv[])
{
    START:
	try {
        return vvr::mainLoop(argc, argv, new POLY_2D);
    }
	catch (std::string exc) {
		cerr << exc << endl;
		return 1;
	}
	catch (...)
	{
		cerr << "Unknown exception" << endl;
		return 1;
	}
}

POLY_2D::POLY_2D()
{
    //set the background color
    m_bg_col = vvr::Colour(0, 0, 255);
    reset();
}

void POLY_2D::reset()
{
    Scene::reset();
    const int BW = 200;
    const int BH = 200;
    C2DRect bound(-BW, BH, BW, -BH);
    C2DPolygon rand_poly;
    rand_poly.CreateRandom(bound, MAX_NUM_PTS, MAX_NUM_PTS);
    rand_poly.GetPointsCopy(my_point_cloud);
    polygon_creation(&my_point_cloud, &my_polygon);

    my_mouse_pos = new vvr::Point2D();
    my_camvas.add(my_mouse_pos);
}

void POLY_2D::keyEvent(unsigned char key, bool up, int modif)
{
    key = tolower(key);

    switch (key)
    {
    case '2': if (!m_run_task_2) {
        m_run_task_2 = true;
        task_2(&my_polygon);break;
    }
            else {
        my_camvas_task_2.clear();
        convex_set.RemoveAll();
        //my_camvas_3.clear();
        core_poly.Clear();
        m_run_task_2 = false;
        break;
    }
    case '3': 
        my_camvas_task_2.clear();
        //my_camvas_3.clear();
        m_run_task_2 = false;
        my_point_cloud.RemoveAll();
        my_polyset.RemoveAll();
        po_lineset.RemoveAll();
        inter2_set.RemoveAll();
        convex_set.RemoveAll();
        lineset.RemoveAll();
        core_poly.Clear();
        reset();break;
    }
}

void POLY_2D::draw()
{
    enterPixelMode();
    my_camvas_task_2.draw();    
    //my_camvas_3.draw();
    vvr::draw(my_polygon, vvr::Colour::green);
    vvr::draw(my_vision_polygon, vvr::Colour::black);
    vvr::draw(core_poly, vvr::Colour::black);
    vvr::draw(po_lineset, vvr::Colour::grey);
    returnFromPixelMode();
}

void POLY_2D::ConvexHull_Fast()
{
    C2DPoint* pts = new C2DPoint[convex_set.size()];
    for (int i = 0; i < convex_set.size(); i++)
        pts[i] = *(convex_set.GetAt(i));
    C2DPolygon cloud_polygon;
    cloud_polygon.Create(pts, convex_set.size());
    core_poly.CreateConvexHull(cloud_polygon);
    delete[] pts;
}

int POLY_2D::task_1(const C2DPoint* p1, const C2DPolygon* q)
{
    int position = 0;
    C2DCircle m_circle;
    m_circle = C2DCircle(*p1, 300);
    circleEquation circle1;
    circle1 = createCircleEquation(m_circle);
    C2DPoint p_end;
    for (double theta = 0;theta < 360;theta += 0.5)
    {
        p_end.x = circle1.c * circle1.c * cos(theta * PI / 180.0) + circle1.a;
        p_end.y = circle1.c * circle1.c * sin(theta * PI / 180.0) + circle1.b;
        C2DLine line(*p1, p_end);
        lineset.AddCopy(line);
        C2DPoint intersection_pt;
        C2DPointSet inter_set;
        if (theta >= 0 && theta <= 90)
        {
            for (int i = 0;i < q->GetPointsCount();i++)
            {
                line_intersection_point(createLineEquation(line), createLineEquation(*q->GetLine(i)), intersection_pt);
                if (intersection_pt.x >= circle1.a && intersection_pt.y >= circle1.b)
                {
                    if (lineContainsPoint(createLineEquation(line), intersection_pt) && lineContainsPoint(createLineEquation(*q->GetLine(i)), intersection_pt))
                    {
                        inter_set.AddCopy(intersection_pt);
                    }
                }
            }
            if (inter_set.size() > 0) {
                my_polyset.AddCopy(*inter_set.GetAt(min_dist(&inter_set, *p1)));
            }
            else {
                if (!position && my_polyset.size() > 0) {
                    position = my_polyset.size() - 1;
                }
            }
            inter_set.RemoveAll();
        }
        else if (theta >= 90 && theta <= 180)
        {
            for (int i = 0;i < q->GetPointsCount();i++)
            {
                line_intersection_point(createLineEquation(line), createLineEquation(*q->GetLine(i)), intersection_pt);
                if (intersection_pt.x <= circle1.a && intersection_pt.y >= circle1.b)
                {
                    if (lineContainsPoint(createLineEquation(line), intersection_pt) && lineContainsPoint(createLineEquation(*q->GetLine(i)), intersection_pt))
                    {
                        inter_set.AddCopy(intersection_pt);
                    }
                }
            }
            if (inter_set.size() > 0) {
                my_polyset.AddCopy(*inter_set.GetAt(min_dist(&inter_set, *p1)));
            }
            else {
                if (!position && my_polyset.size() > 0) {
                    position = my_polyset.size() - 1;
                }
            }
            inter_set.RemoveAll();
        }
        else if (theta >= 180 && theta <= 270)
        {
            for (int i = 0;i < q->GetPointsCount();i++)
            {
                line_intersection_point(createLineEquation(line), createLineEquation(*q->GetLine(i)), intersection_pt);
                if (intersection_pt.x <= circle1.a && intersection_pt.y <= circle1.b)
                {
                    if (lineContainsPoint(createLineEquation(line), intersection_pt) && lineContainsPoint(createLineEquation(*q->GetLine(i)), intersection_pt))
                    {
                        inter_set.AddCopy(intersection_pt);
                    }
                }
            }
            if (inter_set.size() > 0) {
                my_polyset.AddCopy(*inter_set.GetAt(min_dist(&inter_set, *p1)));
            }
            else {
                if (!position && my_polyset.size() > 0) {
                    position = my_polyset.size() - 1;
                }
            }
            inter_set.RemoveAll();
        }
        else
        {
            for (int i = 0;i < q->GetPointsCount();i++)
            {
                line_intersection_point(createLineEquation(line), createLineEquation(*q->GetLine(i)), intersection_pt);
                if (intersection_pt.x >= circle1.a && intersection_pt.y <= circle1.b)
                {
                    if (lineContainsPoint(createLineEquation(line), intersection_pt) && lineContainsPoint(createLineEquation(*q->GetLine(i)), intersection_pt))
                    {
                        inter_set.AddCopy(intersection_pt);
                    }
                }
            }
            if (inter_set.size() > 0) {
                my_polyset.AddCopy(*inter_set.GetAt(min_dist(&inter_set, *p1)));
            }
            else {
                if (!position && my_polyset.size() > 0) {
                    position = my_polyset.size() - 1;
                }
            }
            inter_set.RemoveAll();
        }
    }
    my_polyset.RemoveRepeatedPoints();
    return position;
}

void POLY_2D::task_2(const C2DPolygon* q)
{
    intersection_in_polygon(q, &inter2_set);
    for (int i = 0;i < my_point_cloud.size();i++)
    {
        inter2_set.AddCopy(*my_point_cloud.GetAt(i));
    }
    inter2_set.RemoveRepeatedPoints();
    int ar[MAX_NUM_PTS];
    find_sign(q, ar);
    check_sign(q, ar, &inter2_set);
    if (convex_set.size() > 2)
        ConvexHull_Fast();
    else if (convex_set.size() == 2)
    {
        C2DLine line11(*convex_set.GetAt(0), *convex_set.GetAt(1));
        my_camvas_task_2.add(line11, vvr::Colour::black);
    }
    else if (convex_set.size() == 1)
        my_camvas_task_2.add(*convex_set.GetAt(0), vvr::Colour::black);
}

void POLY_2D::check_sign(const C2DPolygon* poly, int array[], C2DPointSet* Set)
{
    for (int j = 0;j < Set->size();j++)
    {
        bool res = true;
        C2DPoint p = *Set->GetAt(j);
        for (int i = 0;i < poly->GetLineCount();i++)
        {
            if (!(IsPositive(createLineEquation(*poly->GetLine(i)), p) == array[i] || check_pointinline(createLineEquation(*poly->GetLine(i)), p)))
            {
                res = false;
                break;
            }
        }
        if (res)
            convex_set.AddCopy(p);
    }
}

bool POLY_2D::check_pointinline(lineEquation line, C2DPoint p)
{
    double error = 0.000001;
    if ((line.a * p.x + line.b * p.y + line.c <= error) && (line.a * p.x + line.b * p.y + line.c >= -error))
        //if (line.a * p.x + line.b * p.y + line.c == 0)
        return true;
    return false;
}

void POLY_2D::find_sign(const C2DPolygon* poly, int array[])
{
    for (int i = 0;i < poly->GetLineCount();i++)
    {
        C2DPoint mid_p = poly->GetLine(i)->GetMidPoint();
        lineEquation line = createLineEquation(*poly->GetLine(i));
        lineEquation vline = createverticalLineEquation(*poly->GetLine(i), mid_p);
        C2DPoint i1, i2;
        C2DCircle m_circle = C2DCircle(mid_p, 0.01);
        circleEquation circle1 = createCircleEquation(m_circle);
        getPointIntersection(circle1, vline, i1, i2);
        my_camvas.add(i1, vvr::Colour::red);
        my_camvas.add(i2, vvr::Colour::yellow);
        if (poly->Contains(i1))
        {
            if (IsPositive(line, i1))
                array[i] = 1;
            else
                array[i] = 0;
        }
        else if (poly->Contains(i2))
        {
            if (IsPositive(line, i2))
                array[i] = 1;
            else
                array[i] = 0;
        }
    }
}

bool POLY_2D::IsPositive(const lineEquation line, C2DPoint p)
{
    if (line.a * p.x + line.b * p.y + line.c > 0)
        return true;
    else if (line.a * p.x + line.b * p.y + line.c < 0)
        return false;
}

void POLY_2D::intersection_in_polygon(const C2DPolygon* poly, C2DPointSet* Set)
{
    for (int i = 0;i < poly->GetLineCount();i++)
    {
        lineEquation line_1 = createLineEquation(*poly->GetLine(i));
        for (int j = 0;j < poly->GetLineCount();j++)
        {
            if (i == j) continue;
            lineEquation line_2 = createLineEquation(*poly->GetLine(j));
            // if (line_1.a == line_2.a && line_1.b == line_2.b && line_1.c == line_2.c) continue;
            C2DPoint intersection_pt;
            line_intersection_point(line_1, line_2, intersection_pt);
            if (poly->Contains(intersection_pt))
                Set->AddCopy(intersection_pt);
        }
    }
}

bool POLY_2D::lineContainsPoint(const lineEquation& line, const C2DPoint p)
{
    C2DPoint p1 = line.line.GetPointTo();
    C2DPoint p2 = line.line.GetPointFrom();
    double dist_p1 = p1.Distance(p);
    double dist_p2 = p2.Distance(p);
    double linedist = p1.Distance(p2);
    if (dist_p1 > linedist || dist_p2 > linedist) {
        return false;
    }
    return true;
}

int POLY_2D::min_dist(C2DPointSet* Set, C2DPoint p)
{
    double min = p.Distance(*Set->GetAt(0));
    int position = 0;
    for (int i = 1;i < Set->size();i++)
    {
        if (p.Distance(*Set->GetAt(i)) < min)
        {
            min = p.Distance(*Set->GetAt(i));
            position = i;
        }
    }
    return position;
}

void POLY_2D::mousePressed(int x, int y, int modif)
{
    my_mouse_pos->setColour(vvr::Colour::magenta);
    my_mouse_pos->x = x; my_mouse_pos->y = y;
    C2DPoint p(x, y);
    my_polyset.RemoveAll();
    po_lineset.RemoveAll();
    lineset.RemoveAll();
    int position=task_1(&p, &my_polygon);
    tr_pointset_to_lineset(&po_lineset, &my_polyset,position);
}


void POLY_2D::mouseReleased(int x, int y, int modif)
{
    //my_camvas_3.clear();
    my_mouse_pos->setColour(vvr::Colour::white);
    my_mouse_pos->x = x; my_mouse_pos->y = y;
    C2DPoint p(x, y);
    //my_camvas_3.add(p);
}

void  POLY_2D::mouseMoved(int x, int y, int modif)
{
    Scene::mouseMoved(x, y, modif);
    mousePressed(x, y, modif);
}

POLY_2D::lineEquation POLY_2D::createLineEquation(const C2DLine& line) {
    POLY_2D::lineEquation equation;
    C2DPoint p1, p2;
    p1 = line.GetPointTo();
    p2 = line.GetPointFrom();

    // ax + by + c = 0
    if (p2.x - p1.x != 0) {
        equation.a = -(p2.y - p1.y) / (p2.x - p1.x);
        equation.b = 1;
        equation.c = -equation.a * p1.x - p1.y;
    }
    else
    {
        equation.a = 1;
        equation.b = 0;
        equation.c = -p2.x;
    }
    equation.line = line;
    return equation;
}

POLY_2D::circleEquation POLY_2D::createCircleEquation(const C2DCircle& circle)
{
    //(x-a)^2 + (y-b)^2 = c^2
    circleEquation equation;
    equation.a = circle.GetCentre().x;
    equation.b = circle.GetCentre().y;
    equation.c = circle.GetRadius();
    equation.circle = circle;
    return equation;
}
void POLY_2D::line_intersection_point(const POLY_2D::lineEquation line1, const POLY_2D::lineEquation line2, C2DPoint& intersection_pt) {
    if (line1.b == 0 && line2.a == 0)
    {
        intersection_pt.x = -line1.c;
        intersection_pt.y = -line2.c;
    }
    else if (line1.a == 0 && line2.b == 0)
    {
        intersection_pt.x = -line2.c;
        intersection_pt.y = -line1.c;
    }
    else if (line1.b == 0)
    {
        intersection_pt.x = -line1.c;
        intersection_pt.y = -line2.a * intersection_pt.x - line2.c;
    }
    else if (line2.b == 0)
    {
        intersection_pt.x = -line2.c;
        intersection_pt.y = -line1.a * intersection_pt.x - line1.c;
    }
    else if (line1.a - line2.a != 0)
    {
        intersection_pt.x = (line2.c - line1.c) / (line1.a - line2.a);
        intersection_pt.y = -line1.a * intersection_pt.x - line1.c;
    }
}

POLY_2D::lineEquation POLY_2D::createverticalLineEquation(const C2DLine& line, const C2DPoint& point) {
    POLY_2D::lineEquation equation;
    C2DPoint p1, p2;
    p1 = line.GetPointTo();
    p2 = line.GetPointFrom();
    // ax + by+c=0
    if (p2.y - p1.y != 0) {
        equation.a = (p2.x - p1.x) / (p2.y - p1.y);
        equation.b = 1;
        equation.c = -equation.a * point.x - point.y;
    }
    else
    {
        equation.a = 1;
        equation.b = 0;
        equation.c = -point.x;
    }
    equation.line = line;
    return equation;
}

void POLY_2D::getPointIntersection(const circleEquation circle, const lineEquation line, C2DPoint& point1, C2DPoint& point2)
{
    if (line.a != 0)
    {
        double A = line.b / line.a;
        double B = line.c / line.a;

        double w1 = (2 * A * B + 2 * A * circle.a - 2 * circle.b) / (A * A + 1);
        double w2 = (circle.a * circle.a + circle.b * circle.b + B * B + 2 * B * circle.a - circle.c * circle.c) / (A * A + 1);
        double D = sqrt(w1 * w1 - 4 * w2);
        point1.y = (-w1 - D) / 2;
        point2.y = (-w1 + D) / 2;

        point1.x = -A * point1.y - B;
        point2.x = -A * point2.y - B;
    }
    else
    {
        double w1 = -2 * circle.a;
        double w2 = circle.a * circle.a + (line.c + circle.b) * (line.c + circle.b) - circle.c * circle.c;
        double D = sqrt(w1 * w1 - 4 * w2);
        point1.y = -line.c;
        point2.y = -line.c;
        point1.x = (-w1 - D) / 2;
        point2.x = (-w1 + D) / 2;
    }
}
void POLY_2D::tr_pointset_to_lineset(C2DLineSet* LSet, C2DPointSet* PSet,int position)
{
    for (int i = 0;i < PSet->size();i++)
    {
        if (i == position && i>0)continue;
        LSet->AddCopy(*PSet->GetAt(i), *PSet->GetAt((i+1)% PSet->size()));
    }

}

void POLY_2D::polygon_creation(C2DPointSet* set, C2DPolygon* poly)
{
    poly->Create(*set, true);
}
    


