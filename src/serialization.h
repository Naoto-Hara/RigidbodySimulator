#ifndef serialization_h
#define serialization_h

#define _USE_MATH_DEFINES
#include <Eigen/Core>
#include <vector>
#include <fstream>
#include "cereal_eigen.h"
#include <cereal/cereal.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>

#pragma warning(disable: 4996)
struct Simulator;

struct ObjectInitInfo
{
    double scale;
    bool isStatic;
    int template_index;

    Eigen::Vector2d x;
    Eigen::Vector2d v;
    double theta;
    double omega;

    double density;

    int type;
};

struct ShapeTemplates
{
    void setTemplate(const std::vector< Eigen::Vector2d >& in_polygon_template) {
        polygon_templates.push_back(in_polygon_template);
    }

    void setTemplate(const double in_circle_template) {
        circle_templates.push_back(in_circle_template);
    }

    void clearData()
    {
        polygon_templates.clear();
        circle_templates.clear();
    }

    template<class Archive>
    void serialize(Archive& archive)
    {
        archive(CEREAL_NVP(polygon_templates),
            CEREAL_NVP(circle_templates));
    }

    void serialize(const char* file_name)
    {
        std::ofstream out(std::string(file_name), std::ios::binary);
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(CEREAL_NVP(*this));
    }

    void serialize(const std::string& file_name)
    {
        std::ofstream out(file_name, std::ios::binary);
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(CEREAL_NVP(*this));
    }

    int numTemplates() const { return polygon_templates.size() + circle_templates.size(); }

    std::vector< std::vector< Eigen::Vector2d > > polygon_templates;
    std::vector< double > circle_templates;
};



struct ObjectList
{
    ObjectList()
        : time(0.0), frame_idx(0), auto_save_idx(0), prev_auto_save_time(-1000.0)
    {}

    void pushBackObject(const ObjectInitInfo& in_Obj)
    {
        isStatic_list.push_back(in_Obj.isStatic);
        type_list.push_back(in_Obj.type);
        template_index_list.push_back(in_Obj.template_index);
        density_list.push_back(in_Obj.density);
        scale_list.push_back(in_Obj.scale);
        x_list.push_back(in_Obj.x);
        v_list.push_back(in_Obj.v);
        theta_list.push_back(in_Obj.theta);
        omega_list.push_back(in_Obj.omega);
    }

    void print() {

    }

    //’Ç‹L
    void eraseData(int num) {
        isStatic_list.erase(isStatic_list.begin() + num);
        type_list.erase(type_list.begin() + num);
        template_index_list.erase(template_index_list.begin() + num);
        density_list.erase(density_list.begin() + num);
        scale_list.erase(scale_list.begin() + num);
        x_list.erase(x_list.begin() + num);
        v_list.erase(v_list.begin() + num);
        theta_list.erase(theta_list.begin() + num);
        omega_list.erase(omega_list.begin() + num);
    }
    //

    void clearData()
    {
        isStatic_list.clear();
        type_list.clear();
        template_index_list.clear();
        density_list.clear();
        scale_list.clear();
        x_list.clear();
        v_list.clear();
        theta_list.clear();
        omega_list.clear();
    }

    int numObjects() const
    {
        return x_list.size();
    }

    template<class Archive>
    void serialize(Archive& archive)
    {
        archive(CEREAL_NVP(isStatic_list), CEREAL_NVP(type_list), CEREAL_NVP(template_index_list),
            CEREAL_NVP(density_list), CEREAL_NVP(scale_list), CEREAL_NVP(x_list),
            CEREAL_NVP(v_list), CEREAL_NVP(theta_list), CEREAL_NVP(omega_list),
            CEREAL_NVP(time), CEREAL_NVP(frame_idx), CEREAL_NVP(auto_save_idx), CEREAL_NVP(prev_auto_save_time));
    }

    void serialize(const char* file_name)
    {
        std::ofstream out(std::string(file_name), std::ios::binary);
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(CEREAL_NVP(*this));
    }

    void serialize(const std::string& file_name)
    {
        std::ofstream out(file_name, std::ios::binary);
        cereal::BinaryOutputArchive archive_o(out);
        archive_o(CEREAL_NVP(*this));
    }

    std::vector< bool > isStatic_list;
    std::vector< int > type_list;
    std::vector< int > template_index_list;

    std::vector< double > density_list;
    std::vector< double > scale_list;
    std::vector< Eigen::Vector2d > x_list;
    std::vector< Eigen::Vector2d > v_list;
    std::vector< double > theta_list;
    std::vector< double > omega_list;

    double time;
    int frame_idx;
    int auto_save_idx;
    double prev_auto_save_time;
};

void serializeSim(const std::string& in_Templates_filename, const std::string& in_Objects_filename, ShapeTemplates& in_Templates, ObjectList& in_Objects);
void serializeSim(const std::string& in_Templates_filename, const std::string& in_Objects_filename, const Simulator& in_Simulator);
void serializeObj(const std::string& in_Objects_filename, ObjectList& in_Objects);
void serializeObj(const std::string& in_Objects_filename, const Simulator& in_Simulator);
void deserializeSim(const std::string& in_Templates_filename, const std::string& in_Objects_filename, ShapeTemplates& out_Templates, ObjectList& out_Objects);
void initializeSimulator(const ShapeTemplates& in_Templates, const ObjectList& in_ObjectList, Simulator& io_Simulator);

#endif

