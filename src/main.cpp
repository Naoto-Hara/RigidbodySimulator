#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DONT_VECTORIZE

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#include <GLUT/glut.h>
#include <OpenGL/OpenGL.h>
#else
#include <GL/glut.h>
#endif

#pragma warning(disable: 4996)

#include <iostream>
#include <vector>
#include <random>
#include <Eigen/Core>
#include "rigidbody2d.h"
#include "polygonobj.h"
#include "circleobj.h"
#include "signeddistance2d.h"
#include "simulator.hpp"
#include "constants.h"
#include "collisiondetection2d.h"
#include "xmlparser.h"
#include "serialization.h"
#include <Eigen/Dense>
#include <string>
#include <filesystem>
#include <Windows.h>


std::random_device rnd;
std::mt19937 mt(rnd());
std::uniform_real_distribution<> randomMT(0.0, 1.0);

Simulator g_Simulator;

void getBoundingBox(const std::vector< Eigen::Vector2d >& in_vertices0, BoundingBox2D& out_bb);
void screenshotcereal(int num);

void pre_display(const Simulator& in_Simulator)
{
    glViewport(0, 0, in_Simulator.window_width * in_Simulator.frame_window_size_scale_x, in_Simulator.window_height * in_Simulator.frame_window_size_scale_y);

    glClearColor(1.0, 1.0, 1.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    const int height_reference = 540;
    const double half_width = 2.0 * double(in_Simulator.window_width) / double(height_reference);
    const double half_height = 2.0 * double(in_Simulator.window_height) / double(height_reference);
    glOrtho(-half_width, half_width, -half_height, half_height, -1.0, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void post_display()
{
    glutSwapBuffers();
}

void reshape_func(int width, int height)
{
    g_Simulator.window_height = width;
    g_Simulator.window_height = height;
}


void idle_func()
{
    if (g_Simulator.animation)
    {
        for (int i = 0; i < 100; i++)
            stepSystem(g_Simulator, g_Simulator.settings.dt);

        const double time = g_Simulator.settings.dt * g_Simulator.frame_idx;

        /*
        if( g_Simulator.settings.max_time > 0.0 && time >= g_Simulator.settings.max_time )
        {
          if( !g_Simulator.settings.serialization_folder.empty() && !g_Simulator.settings.templates_file_name_for_serialization.empty() && !g_Simulator.settings.objects_file_name_for_serialization.empty() )
          {
            serializeSim( g_Simulator.settings.serialization_folder + "/" + g_Simulator.settings.templates_file_name_for_serialization, g_Simulator.settings.serialization_folder + "/" + g_Simulator.settings.objects_file_name_for_serialization, g_Simulator );
          }

          std::cout << "Simulation done" << std::endl;
          exit(0);
        }
        */


        if (g_Simulator.settings.auto_save)
        {
            if (time >= g_Simulator.prev_auto_save_time + g_Simulator.settings.auto_save_interval)
            {
                char file_name[1024];
                sprintf(file_name, g_Simulator.settings.intermediate_objects_file_template_for_serialization.c_str(), g_Simulator.auto_save_idx);
                std::string fn = g_Simulator.settings.serialization_folder + "/" + file_name;
                serializeObj(fn, g_Simulator);

                g_Simulator.prev_auto_save_time = time;
                g_Simulator.auto_save_idx++;
                
            }
        }


    }

    glutPostRedisplay();
}

/*
void drawSDF( const RigidBody2D* in_Body )
{
  const SignedDistanceFunction2D* _sdf = in_Body->getSDF();
  glBegin(GL_QUADS);
  for (int j = 0; j <= _sdf->resolution()(1); j++)
  {
    for (int i = 0; i <= _sdf->resolution()(0); i++)
    {
      const double sd00 = _sdf->signedDistance( Eigen::Vector2i{ i, j } );
      const double sd10 = _sdf->signedDistance( Eigen::Vector2i{ i + 1, j } );
      const double sd11 = _sdf->signedDistance( Eigen::Vector2i{ i + 1, j + 1 } );
      const double sd01 = _sdf->signedDistance( Eigen::Vector2i{ i, j + 1 } );


      const Eigen::Vector2d x00_t0 = _sdf->minVertex() + Eigen::Vector2d{ i, j } *_sdf->dx();
      const Eigen::Vector2d x10_t0 = _sdf->minVertex() + Eigen::Vector2d{ i + 1, j } *_sdf->dx();
      const Eigen::Vector2d x01_t0 = _sdf->minVertex() + Eigen::Vector2d{ i, j + 1 } *_sdf->dx();
      const Eigen::Vector2d x11_t0 = _sdf->minVertex() + Eigen::Vector2d{ i + 1, j + 1 } *_sdf->dx();

      const Eigen::Vector2d x00 = in_Body->getCurrentPosition( x00_t0 );
      const Eigen::Vector2d x10 = in_Body->getCurrentPosition( x10_t0 );
      const Eigen::Vector2d x01 = in_Body->getCurrentPosition( x01_t0 );
      const Eigen::Vector2d x11 = in_Body->getCurrentPosition( x11_t0 );

      if (sd00 > 0.0) glColor3d(1.0 - sd00, 1.0 - sd00, 1.0); else glColor3d(1.0, 1.0 + sd00, 1.0 + sd00);
      glVertex2d(x00(0), x00(1));
      if (sd10 > 0.0) glColor3d(1.0 - sd10, 1.0 - sd10, 1.0); else glColor3d(1.0, 1.0 + sd10, 1.0 + sd10);
      glVertex2d(x10(0), x10(1));
      if (sd11 > 0.0) glColor3d(1.0 - sd11, 1.0 - sd11, 1.0); else glColor3d(1.0, 1.0 + sd11, 1.0 + sd11);
      glVertex2d(x11(0), x11(1));
      if (sd01 > 0.0) glColor3d(1.0 - sd01, 1.0 - sd01, 1.0); else glColor3d(1.0, 1.0 + sd01, 1.0 + sd01);
      glVertex2d(x01(0), x01(1));
    }
  }
  glEnd();
}
//*/

void drawSamplePoints(const RigidBody2D* in_Body)
{
    glEnable(GL_POINT_SIZE);
    glPointSize(5.0);
    glBegin(GL_POINTS);

    for (int i = 0; i < in_Body->numCollisionSamples(); i++)
    {
        const Eigen::Vector2d samplePoint0 = in_Body->collisionSample(i).x0;
        const Eigen::Vector2d currentSamplePoint = in_Body->getCurrentPosition(samplePoint0);
        if (!in_Body->collisionSample(i).collision_cache.empty())
            glColor3d(0.0, 1.0, 0.0);
        else
            glColor3d(1.0, 0.8, 0.0);
        glVertex2d(currentSamplePoint(0), currentSamplePoint(1));
    }

    glEnd();

    glLineWidth(2.0);
    glBegin(GL_LINES);
    for (int i = 0; i < in_Body->numCollisionSamples(); i++)
    {
        const Eigen::Vector2d samplePoint0 = in_Body->collisionSample(i).x0;
        const Eigen::Vector2d currentSamplePoint = in_Body->getCurrentPosition(samplePoint0);

        for (auto q = in_Body->collisionSample(i).collision_cache.begin(); q != in_Body->collisionSample(i).collision_cache.end(); q++)
        {
            const Eigen::Vector2d normalTip = currentSamplePoint + q->second.normal * 0.2;
            const Eigen::Vector2d tangentTip = currentSamplePoint + q->second.tangent * 0.2;
            glColor3d(0.0, 0.0, 0.0);
            glVertex2d(currentSamplePoint(0), currentSamplePoint(1));
            glVertex2d(normalTip(0), normalTip(1));
            glColor3d(1.0, 1.0, 0.0);
            glVertex2d(currentSamplePoint(0), currentSamplePoint(1));
            glVertex2d(tangentTip(0), tangentTip(1));
        }
    }
    glEnd();
}

void drawPolygon(const RigidBody2D& in_body)
{
    /*
     double colors[3][3] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };
     const int Ncolors = 3;
     glBegin(GL_TRIANGLES);
     for (int k = 0; k < in_Polygon.numVertices() - 2; k++)
     {
     glColor3d(colors[0][0], colors[0][1], colors[0][2]);
     const Vector2d v0 = in_Polygon.getVertexPosition0(0);
     glVertex2d(v0(0), v0(1));
     glColor3d(colors[1][0], colors[1][1], colors[1][2]);
     const Vector2d v1 = in_Polygon.getVertexPosition0(1);
     glVertex2d(v1(0), v1(1));
     glColor3d(colors[(k + 2) % Ncolors][0], colors[(k + 2) % Ncolors][1], colors[(k+2)%Ncolors][2]);
     const Vector2d vk = in_Polygon.getVertexPosition0(k+2);
     glVertex2d(vk(0), vk(1));
     }
     glEnd();
     //*/

    const PolygonObjTemplate* body_template = reinterpret_cast<const PolygonObjTemplate*>(in_body.getTemplate());

    glEnable(GL_LINE_WIDTH);
    glLineWidth(3.0);
    glBegin(GL_LINE_LOOP);
    for (int k = 0; k < body_template->numVertices(); k++)
    {
        glColor3d(0.0, 0.0, 0.0);
        const Eigen::Vector2d vk = in_body.getCurrentPosition(body_template->getVertexPosition0(k));
        glVertex2d(vk(0), vk(1));
    }
    glEnd();
}

void drawCircle(const RigidBody2D& in_body)
{
    const CircleObjTemplate* body_template = reinterpret_cast<const CircleObjTemplate*>(in_body.getTemplate());

    glEnable(GL_LINE_WIDTH);
    glLineWidth(3.0);
    glBegin(GL_LINE_LOOP);

    const int nSegs = 180;
    const double radius = body_template->getRadius0();
    for (int k = 0; k < nSegs; k++)
    {
        glColor3d(0.0, 0.0, 0.0);
        const double theta = 2.0 * M_PI * (k + 0.5) / nSegs;
        const Eigen::Vector2d _vk{ radius * cos(theta), radius * sin(theta) };
        const Eigen::Vector2d vk = in_body.getCurrentPosition(_vk);
        glVertex2d(vk(0), vk(1));
    }
    glEnd();
}

void drawBorderLine() {
    glEnable(GL_LINE_WIDTH);
    glLineWidth(3.0);
    glBegin(GL_LINES);

    glColor3d(1.0, 0.0, 0.0);
    glVertex2d(-6.15, 0.075);
    glVertex2d(6.15, 0.075);
    glEnd();
}

void copyFile(const char* from_file_name, const char* to_file_name)
{
    //fstream::rdbuf()Ç∆<<ÇégÇ¡ÇΩï˚ñ@
    std::ifstream is(from_file_name, std::ios::in | std::ios::binary);
    std::ofstream os(to_file_name, std::ios::out | std::ios::binary);

    // ÉtÉ@ÉCÉãÉRÉsÅ[
    os << is.rdbuf();
}

/*
void staticWallTemplateSet( const ShapeTemplates& t ) {

  for(int i =0;i < t.wall_templates.size();i++)
  g_Simulator.rigid_bodies.push_back(new PolygonObj(t.wall_templates[i], i, true,t.wall_rho_templates[i], nullptr, t.wall_centerOfMass_templates[i]));

  for (int i = 0; i < t.wall_circle_templates.size(); i++)
  g_Simulator.rigid_bodies.push_back(new CircleObj(t.wall_circle_templates[i], i + t.wall_templates.size(), true, t.wall_circle_rho_templates[i], nullptr, t.wall_circle_centerOfMass_templates[i]));
}
//*/

std::vector<std::string> split(std::string& in, char d)
{
    std::istringstream stream(in);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, d))
    {
        result.push_back(field);
    }
    return result;
}

void display_func()
{
    pre_display(g_Simulator);

    glClear(GL_COLOR_BUFFER_BIT);

    for (int i = 0; i < g_Simulator.rigid_bodies.size(); i++)
    {
        //drawSDF(g_Simulator.rigid_bodies[i]);
        if (g_Simulator.rigid_bodies[i]->getType() == POLYGON)
            drawPolygon(*g_Simulator.rigid_bodies[i]);
        else if (g_Simulator.rigid_bodies[i]->getType() == CIRCLE)
            drawCircle(*g_Simulator.rigid_bodies[i]);
        //drawSamplePoints(g_Simulator.rigid_bodies[i]);
    }

    drawBorderLine();

    post_display();
}





void initialize_1(ShapeTemplates& out_Templates, ObjectList& out_ObjectList)
{
    out_Templates.clearData();
    out_ObjectList.clearData();

    /* Polygon 0 */
    std::vector< Eigen::Vector2d > polygon_0_template;
    polygon_0_template.emplace_back(0.0, 0.5);
    polygon_0_template.emplace_back(-0.5, -0.25);
    polygon_0_template.emplace_back(0.5, -0.25);

    out_Templates.setTemplate(polygon_0_template);

    ObjectInitInfo object_0;
    object_0.isStatic = false;
    object_0.type = POLYGON;
    object_0.template_index = 0;
    object_0.density = 10.0;
    object_0.scale = 0.5;
    object_0.x << 0.3, -0.3;
    object_0.v << 0.0, 0.0;
    object_0.theta = 0.5;
    object_0.omega = 0.0;

    out_ObjectList.pushBackObject(object_0);

    /* Polygon 1 */
    std::vector< Eigen::Vector2d > polygon_1_template;
    polygon_1_template.emplace_back(0.0, 0.0);
    polygon_1_template.emplace_back(0.8, 0.0);
    polygon_1_template.emplace_back(0.8, 0.8);
    polygon_1_template.emplace_back(0.0, 0.8);

    out_Templates.setTemplate(polygon_1_template);

    ObjectInitInfo object_1;
    object_1.isStatic = false;
    object_1.type = POLYGON;
    object_1.template_index = 1;
    object_1.density = 20.0;
    object_1.scale = 0.5;
    object_1.x << -0.3, 0.3;
    object_1.v << 0.0, 0.0;
    object_1.theta = 0.1;
    object_1.omega = 0.0;

    out_ObjectList.pushBackObject(object_1);

    /* Circle 0 */
    double circle_0_template;
    circle_0_template = 0.3;

    out_Templates.setTemplate(circle_0_template);

    ObjectInitInfo object_2;
    object_2.isStatic = false;
    object_2.type = CIRCLE;
    object_2.template_index = 0;
    object_2.density = 20.0;
    object_2.scale = 0.5;
    object_2.x << 0.3, 0.3;
    object_2.v << 0.0, 0.0;
    object_2.theta = 0.0;
    object_2.omega = 0.0;

    out_ObjectList.pushBackObject(object_2);

    /* Polygon 2 (Static Wall 0) */
    std::vector< Eigen::Vector2d > polygon_2_template_static_wall_0;
    polygon_2_template_static_wall_0.emplace_back(0.0, 0.0);
    polygon_2_template_static_wall_0.emplace_back(0.3, 0.0);
    polygon_2_template_static_wall_0.emplace_back(0.3, 4.0);
    polygon_2_template_static_wall_0.emplace_back(0.0, 4.0);

    out_Templates.setTemplate(polygon_2_template_static_wall_0);

    ObjectInitInfo object_3;
    object_3.isStatic = true;
    object_3.type = POLYGON;
    object_3.template_index = 2;
    object_3.density = 100000.0;
    object_3.scale = 1.0;
    object_3.x << -1.5, 0.0;
    object_3.v << 0.0, 0.0;
    object_3.theta = 0.0;
    object_3.omega = 0.0;

    out_ObjectList.pushBackObject(object_3);

    /* Polygon 3 (Static Wall 1) */
    std::vector< Eigen::Vector2d > polygon_3_template_static_wall_1;
    polygon_3_template_static_wall_1.emplace_back(0.0, 0.0);
    polygon_3_template_static_wall_1.emplace_back(0.3, 0.0);
    polygon_3_template_static_wall_1.emplace_back(0.3, 4.0);
    polygon_3_template_static_wall_1.emplace_back(0.0, 4.0);

    out_Templates.setTemplate(polygon_3_template_static_wall_1);

    ObjectInitInfo object_4;
    object_4.isStatic = true;
    object_4.type = POLYGON;
    object_4.template_index = 3;
    object_4.density = 100000.0;
    object_4.scale = 1.0;
    object_4.x << 1.5, 0.0;
    object_4.v << 0.0, 0.0;
    object_4.theta = 0.0;
    object_4.omega = 0.0;

    out_ObjectList.pushBackObject(object_4);

    /* Polygon 4 (Static Wall 2) */
    std::vector< Eigen::Vector2d > polygon_4_template_static_wall_2;
    polygon_4_template_static_wall_2.emplace_back(0.0, 0.0);
    polygon_4_template_static_wall_2.emplace_back(3.0, 0.0);
    polygon_4_template_static_wall_2.emplace_back(3.0, 0.3);
    polygon_4_template_static_wall_2.emplace_back(0.0, 0.3);

    out_Templates.setTemplate(polygon_4_template_static_wall_2);

    ObjectInitInfo object_5;
    object_5.isStatic = true;
    object_5.type = POLYGON;
    object_5.template_index = 4;
    object_5.density = 100000.0;
    object_5.scale = 1.0;
    object_5.x << 0.0, -2.0;
    object_5.v << 0.0, 0.0;
    object_5.theta = 0.0;
    object_5.omega = 0.0;

    out_ObjectList.pushBackObject(object_5);
}

void initTest_1()
{
    ShapeTemplates shape_templates;
    ObjectList object_list;

    initialize_1(shape_templates, object_list);
    initializeSimulator(shape_templates, object_list, g_Simulator);
}

void getBoundingBox(const std::vector< Eigen::Vector2d >& in_vertices0, BoundingBox2D& out_bb)
{
    Eigen::Vector2d center_of_mass = Eigen::Vector2d::Zero();

    for (int i = 0; i < in_vertices0.size(); i++)
    {
        center_of_mass(0) += in_vertices0[i](0);
        center_of_mass(1) += in_vertices0[i](1);
    }
    center_of_mass(0) /= in_vertices0.size();
    center_of_mass(1) /= in_vertices0.size();

    double radius = 0.0;
    for (int i = 0; i < in_vertices0.size(); i++)
    {
        radius = std::max<double>(radius, (in_vertices0[i] - center_of_mass).norm());
    }

    out_bb.bb_min << -radius, -radius;
    out_bb.bb_max << radius, radius;
}

bool overlapTest(const std::vector< BoundingBox2D >& bb_list, const BoundingBox2D& bb)
{
    for (int i = 0; i < bb_list.size(); i++)
    {
        if (boundingBoxIntersection(bb_list[i], bb))
            return false;
    }
    return true;
}

void initialize_2(ShapeTemplates& out_Templates, ObjectList& out_ObjectList)
{
    out_Templates.clearData();
    out_ObjectList.clearData();

    std::vector< BoundingBox2D > template_bbs;

    // Circle 0
    double circle_0_template;
    circle_0_template = 0.05;

    out_Templates.setTemplate(circle_0_template);

    template_bbs.emplace_back(Eigen::Vector2d{ -circle_0_template, -circle_0_template }, Eigen::Vector2d{ circle_0_template, circle_0_template });

    // Polygon 0
    std::vector< Eigen::Vector2d > polygon_0_template;
    double rate = 1.0;
    polygon_0_template.emplace_back(0.0, 0.1 * rate);
    polygon_0_template.emplace_back(-0.1 * rate, -0.05 * rate);
    polygon_0_template.emplace_back(0.1 * rate, -0.05 * rate);

    out_Templates.setTemplate(polygon_0_template);

    BoundingBox2D bb_polygon_0_template; getBoundingBox(polygon_0_template, bb_polygon_0_template);
    template_bbs.push_back(bb_polygon_0_template);

    // Polygon 1
    std::vector< Eigen::Vector2d > polygon_1_template;
    // polygon_1_template.emplace_back( 0.0, 0.0 );
    // polygon_1_template.emplace_back( 0.2, 0.0 );
    // polygon_1_template.emplace_back( 0.2, 0.2 );
    // polygon_1_template.emplace_back( 0.0, 0.2 );
    rate = 1.0;
    polygon_1_template.emplace_back(0.0, 0.0);
    polygon_1_template.emplace_back(0.125 * rate, 0.0);
    polygon_1_template.emplace_back(0.125 * rate, 0.125 * rate);
    polygon_1_template.emplace_back(0.0, 0.125 * rate);

    out_Templates.setTemplate(polygon_1_template);

    BoundingBox2D bb_polygon_1_template; getBoundingBox(polygon_1_template, bb_polygon_1_template);
    template_bbs.push_back(bb_polygon_1_template);

    // Polygon 2
    std::vector< Eigen::Vector2d > polygon_2_template;
    rate = 1.0;
    polygon_2_template.emplace_back(0.0, 0.0);
    polygon_2_template.emplace_back(0.2 * rate, 0.0);
    polygon_2_template.emplace_back(0.2 * rate, 0.08 * rate);
    polygon_2_template.emplace_back(0.08 * rate, 0.08 * rate);
    polygon_2_template.emplace_back(0.08 * rate, 0.15 * rate);
    polygon_2_template.emplace_back(0.0, 0.15 * rate);

    out_Templates.setTemplate(polygon_2_template);

    BoundingBox2D bb_polygon_2_template; getBoundingBox(polygon_2_template, bb_polygon_2_template);
    template_bbs.push_back(bb_polygon_2_template);

    // Polygon 3
    std::vector< Eigen::Vector2d > polygon_3_template;
    polygon_3_template.emplace_back(0.0, 0.0);
    polygon_3_template.emplace_back(0.2, 0.0);
    polygon_3_template.emplace_back(0.2, 0.15);
    polygon_3_template.emplace_back(0.17, 0.15);
    polygon_3_template.emplace_back(0.17, 0.03);
    polygon_3_template.emplace_back(0.03, 0.03);
    polygon_3_template.emplace_back(0.03, 0.15);
    polygon_3_template.emplace_back(0.0, 0.15);

    out_Templates.setTemplate(polygon_3_template);

    BoundingBox2D bb_polygon_3_template; getBoundingBox(polygon_3_template, bb_polygon_3_template);
    template_bbs.push_back(bb_polygon_3_template);

    const double inner_chamber_width = 6.3 * 2;
    const double wall_thickness = 0.3;
    const double wall_left_position = -0.5 * inner_chamber_width - 0.5 * wall_thickness;
    const double wall_right_position = wall_left_position + inner_chamber_width + wall_thickness;
    const double separator_wall_thickness = 0.05;
    const double separator_wall_position = wall_left_position + 1.5 + 0.109;

    // Polygon 4 (for left right walls)
    std::vector< Eigen::Vector2d > polygon_4_template_wall;
    polygon_4_template_wall.emplace_back(0.0, 0.0);
    polygon_4_template_wall.emplace_back(wall_thickness, 0.0);
    polygon_4_template_wall.emplace_back(wall_thickness, 4.3);
    polygon_4_template_wall.emplace_back(0.0, 4.3);

    out_Templates.setTemplate(polygon_4_template_wall);

    BoundingBox2D bb_polygon_4_template; getBoundingBox(polygon_4_template_wall, bb_polygon_4_template);
    template_bbs.push_back(bb_polygon_4_template);

    // Polygon 5 (separator Wall)
    std::vector< Eigen::Vector2d > polygon_5_separator_wall;
    polygon_5_separator_wall.emplace_back(0.0, 0.0);
    polygon_5_separator_wall.emplace_back(separator_wall_thickness, 0.0);
    polygon_5_separator_wall.emplace_back(separator_wall_thickness, 4.3);
    polygon_5_separator_wall.emplace_back(0.0, 4.3);

    out_Templates.setTemplate(polygon_5_separator_wall);

    BoundingBox2D bb_polygon_5_template; getBoundingBox(polygon_5_separator_wall, bb_polygon_5_template);
    template_bbs.push_back(bb_polygon_5_template);


    ObjectInitInfo object_0;
    object_0.isStatic = true;
    object_0.type = POLYGON;
    object_0.template_index = 4;
    object_0.density = 100000.0;
    object_0.scale = 1.0;
    object_0.x << wall_left_position, 0.0;
    object_0.v << 0.0, 0.0;
    object_0.theta = 0.0;
    object_0.omega = 0.0;

    out_ObjectList.pushBackObject(object_0);

    ObjectInitInfo object_1;
    object_1.isStatic = true;
    object_1.type = POLYGON;
    object_1.template_index = 4;
    object_1.density = 100000.0;
    object_1.scale = 1.0;
    object_1.x << wall_right_position, 0.0;
    object_1.v << 0.0, 0.0;
    object_1.theta = 0.0;
    object_1.omega = 0.0;

    out_ObjectList.pushBackObject(object_1);

    ObjectInitInfo object_2;
    object_2.isStatic = true;
    object_2.type = POLYGON;
    object_2.template_index = 5;
    object_2.density = 100000.0;
    object_2.scale = 1.0;
    object_2.x << separator_wall_position, 0.0;
    object_2.v << 0.0, 0.0;
    object_2.theta = 0.0;
    object_2.omega = 0.0;

    out_ObjectList.pushBackObject(object_2);

    // static wall sphare

    double xpos = wall_left_position + wall_thickness * 0.5;
    while (1)
    {
        const double dev = 0.1;
        const double scale = 1.0 + (randomMT(mt) - 0.5) * dev;
        const double radius = circle_0_template * scale;

        ObjectInitInfo object_f;
        object_f.isStatic = true;
        object_f.type = CIRCLE;
        object_f.template_index = 0;
        object_f.density = 100000.0;
        object_f.scale = scale;
        object_f.x << radius + xpos, -2.0;
        object_f.v << 0.0, 0.0;
        object_f.theta = 0.0;
        object_f.omega = 0.0;

        out_ObjectList.pushBackObject(object_f);

        xpos += 2.0 * radius;

        if (xpos > wall_right_position - wall_thickness * 0.5)
            break;
    }

    //inputTemplatePDate(tp, "Save/templatePedeta.cereal");


    const int num_types = 3;
    const int num_total_objects = 150;
    const int max_fail = 1800*10;

    const Eigen::Vector2d bb_min_domain{ wall_left_position + wall_thickness * 0.5, -1.97 };
    const Eigen::Vector2d bb_max_domain{ separator_wall_position - separator_wall_thickness * 0.5,5.0 };

    int num_fail = 0;
    int object_count = 0;

    std::vector< BoundingBox2D > instance_bb_list;

    while (1)
    {
        //int type = 2;
        double t = randomMT(mt);
        int type = std::min<int>( num_types - 1, std::max<int>( 0, int( floor( t * num_types ) ) ) );

        const double dev = 0.1;
        const double scale = 1.0 + (randomMT(mt) - 0.5) * dev;

        const Eigen::Vector2d bb_halfwidth = scale * (template_bbs[type].bb_max - template_bbs[type].bb_min) * 0.5;

        // This is just an approximation (with no check for rotation)
        const Eigen::Vector2d rm = { bb_min_domain(0) + bb_halfwidth(0), bb_min_domain(1) + bb_halfwidth(1) };
        const Eigen::Vector2d rM = { bb_max_domain(0) - bb_halfwidth(0), bb_max_domain(1) - bb_halfwidth(1) };
        const Eigen::Vector2d sz = { rM(0) - rm(0), rM(1) - rm(1) };
        const Eigen::Vector2d center_of_mass = Eigen::Vector2d{ rm(0) + randomMT(mt) * sz(0), rm(1) + randomMT(mt) * sz(1) };
        const double theta = randomMT(mt) * 2.0 * M_PI;

        BoundingBox2D instance_bb(center_of_mass - bb_halfwidth, center_of_mass + bb_halfwidth);

        if (!overlapTest(instance_bb_list, instance_bb))
        {
            num_fail++;
            if (num_fail >= max_fail) break;
            continue;
        }

        instance_bb_list.push_back(instance_bb);

        ObjectInitInfo object;
        object.isStatic = false;
        object.type = (type == 0) ? CIRCLE : POLYGON;
        object.template_index = (type == 0) ? 0 : type - 1;
        object.density = 23.90;
        object.scale = scale;
        object.x = center_of_mass;
        object.v << 0.0, 0.0;
        object.theta = theta;
        object.omega = 0.0;

        out_ObjectList.pushBackObject(object);

        object_count++;

        if (object_count >= num_total_objects) break;

        num_fail = 0;
    }

}

void initTest_2()
{
    ShapeTemplates shape_templates;
    ObjectList object_list;

    initialize_2(shape_templates, object_list);
    initializeSimulator(shape_templates, object_list, g_Simulator);

    if (!g_Simulator.settings.serialization_folder.empty() && !g_Simulator.settings.templates_file_name_for_serialization.empty() && !g_Simulator.settings.objects_file_name_for_serialization.empty())
    {
        serializeSim(g_Simulator.settings.serialization_folder + "/" + g_Simulator.settings.templates_file_name_for_serialization, g_Simulator.settings.serialization_folder + "/" + g_Simulator.settings.objects_file_name_for_serialization, g_Simulator);
    }
}


void resumeSimulation()
{
    ShapeTemplates shape_templates;
    ObjectList object_list;

    deserializeSim(g_Simulator.settings.resume_folder + "/" + g_Simulator.settings.templates_file_name_to_resume, g_Simulator.settings.resume_folder + "/" + g_Simulator.settings.objects_file_name_to_resume, shape_templates, object_list);
    std::cout << "deserialized " << shape_templates.numTemplates() << " templates" << std::endl;
    std::cout << "deserialized " << object_list.numObjects() << " objects" << std::endl;
    initializeSimulator(shape_templates, object_list, g_Simulator);
}


void deleteWall() {
    ShapeTemplates shape_templates;
    ObjectList object_list;

    deserializeSim(g_Simulator.settings.resume_folder + "/" + g_Simulator.settings.templates_file_name_to_resume, g_Simulator.settings.resume_folder + "/" + g_Simulator.settings.objects_file_name_to_resume, shape_templates, object_list);
    std::cout << "deserialized " << shape_templates.numTemplates() << " templates" << std::endl;
    std::cout << "deserialized " << object_list.numObjects() << " objects" << std::endl;
    object_list.eraseData(2);
    initializeSimulator(shape_templates, object_list, g_Simulator);
}

void deleteObject() {
    ShapeTemplates shape_templates;
    ObjectList object_list;

    deserializeSim(g_Simulator.settings.resume_folder + "/" + g_Simulator.settings.templates_file_name_to_resume, g_Simulator.settings.resume_folder + "/" + g_Simulator.settings.objects_file_name_to_resume, shape_templates, object_list);
    std::cout << "deserialized " << shape_templates.numTemplates() << " templates" << std::endl;
    std::cout << "deserialized " << object_list.numObjects() << " objects" << std::endl;

    int num = object_list.numObjects();
    std::vector<int> erase_num;

    for (int i = 0; i < num; i++) {
        if (!(object_list.isStatic_list[i]) && (object_list.x_list[i](1) > 0.075)) {
            erase_num.push_back(i);
        }
    }
    int delete_num = 0;
    for (int i = 0; i < erase_num.size(); i++) {
        object_list.eraseData(erase_num[i] - delete_num);
        delete_num++;
    }

    initializeSimulator(shape_templates, object_list, g_Simulator);
    if (!g_Simulator.settings.serialization_folder.empty() && !g_Simulator.settings.templates_file_name_for_serialization.empty() && !g_Simulator.settings.objects_file_name_for_serialization.empty())
    {
        serializeSim(g_Simulator.settings.serialization_folder + "/" + g_Simulator.settings.templates_file_name_for_serialization, g_Simulator.settings.serialization_folder + "/" + g_Simulator.settings.objects_file_name_for_serialization, g_Simulator);
    }
}




void key(unsigned char key, int x, int y)
{
    switch (key)
    {
    case 'r':
    case 'R':
        //initTest_2();
        //g_Simulator.animation = !g_Simulator.animation;
        //glutPostRedisplay();
        break;
    case ' ':
        g_Simulator.animation = !g_Simulator.animation;
        if (g_Simulator.animation) { std::cout << "simulating..." << std::endl; }
        else { std::cout << "paused" << std::endl; }
        glutPostRedisplay();
        break;
    case 's':
    case 'S':
        //saveObj();
        break;
    case 'l':
    case 'L':
        
        break;
    case 'd':
    case 'D':
        deleteWall();
        break;
    case 'o':
    case 'O':
        deleteObject();
        break;
    case 't':
    case 'T':
        //inputTestLoadFile();
        break;
    case 'c':
    case 'C':
        //initTest_2();
        //g_Simulator.animation = false;
        break;

    }
}


int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " <xml file>" << std::endl;
        exit(0);
    }

    openXMLFile(argv[1], g_Simulator.settings);

    if (!g_Simulator.settings.resume_folder.empty() && !g_Simulator.settings.templates_file_name_to_resume.empty() && !g_Simulator.settings.objects_file_name_to_resume.empty())
    {
        std::cout << "resume" << std::endl;
        //resumeSimulation();
        initTest_2();
    }
    else
    {
        initTest_2();
    }

    glutInit(&argc, argv);

    g_Simulator.window_width = 2000;
    g_Simulator.window_height = 800;
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(g_Simulator.window_width, g_Simulator.window_height);
    glutCreateWindow("AGRigidBody2D");

    // With retina display, frame buffer size is twice the window size.
    // Viewport size should be set on the basis of the frame buffer size, rather than the window size.
    // g_FrameSize_WindowSize_Scale_x and g_FrameSize_WindowSize_Scale_y account for this factor.
    GLint dims[4] = { 0 };
    glGetIntegerv(GL_VIEWPORT, dims);
    g_Simulator.frame_window_size_scale_x = double(dims[2]) / double(g_Simulator.window_width);
    g_Simulator.frame_window_size_scale_y = double(dims[3]) / double(g_Simulator.window_height);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();
    glClear(GL_COLOR_BUFFER_BIT);
    glutSwapBuffers();

    glutDisplayFunc(display_func);
    glutReshapeFunc(reshape_func);
    glutIdleFunc(idle_func);
    glutKeyboardFunc(key);


    glutMainLoop();
    return 0;
}
