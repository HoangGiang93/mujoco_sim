// Copyright (c) 2022, Hoang Giang Nguyen - Institute for Artificial Intelligence, University Bremen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mujoco.h"

#include <boost/filesystem.hpp>
#include <cctype>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <ros/package.h>
#include <tinyxml2.h>
#include <urdf/model.h>

// help
const char helpstring[] =
    "\n Usage:  compile infile outfile\n"
    "   infile must be in urdf format\n"
    "   outfile must be in xml format\n\n"
    " Example:  compile model.urdf model.xml\n";

char error[1000];

std::string meshes_path_string;

// model and error
mjModel *m = 0;

// deallocate and print message
int finish(const char *msg = 0)
{
    // deallocated everything
    if (m)
    {
        mj_deleteModel(m);
    }

    // print message
    if (msg)
    {
        std::printf("%s\n", msg);
    }

    return 0;
}

// possible file types
enum
{
    typeUNKNOWN = 0,
    typeURDF,
    typeXML
};

// determine file type
int filetype(const char *filename)
{
    // convert to lower case for string comparison
    char lower[1000];
    std::size_t i = 0;
    while (i < std::strlen(filename) && i < 999)
    {
        lower[i] = (char)tolower(filename[i]);
        i++;
    }
    lower[i] = 0;

    // find last dot
    int dot = (int)std::strlen(lower);
    while (dot >= 0 && lower[dot] != '.')
    {
        dot--;
    }

    // no dot found
    if (dot < 0)
    {
        return typeUNKNOWN;
    }

    // check extension
    if (!std::strcmp(lower + dot, ".xml"))
    {
        return typeXML;
    }
    else if (!std::strcmp(lower + dot, ".urdf"))
    {
        return typeURDF;
    }
    else
    {
        return typeUNKNOWN;
    }
}

// add_mujoco_tags
void add_mujoco_tags(const boost::filesystem::path &model_urdf_path)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(model_urdf_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        mju_error_s("Couldn't read file in [%s]\n", model_urdf_path.c_str());
    }
    ROS_INFO("path: %s", model_urdf_path.c_str());

    tinyxml2::XMLElement *mujoco = doc.NewElement("mujoco");
    doc.FirstChildElement()->InsertFirstChild(mujoco);
    tinyxml2::XMLElement *compiler = doc.NewElement("compiler");
    mujoco->LinkEndChild(compiler);

    compiler->SetAttribute("meshdir", model_urdf_path.parent_path().c_str());
    compiler->SetAttribute("strippath", false);
    compiler->SetAttribute("balanceinertia", true);
    compiler->SetAttribute("discardvisual", true);

    for (tinyxml2::XMLElement *link = doc.FirstChildElement()->FirstChildElement();
         link != nullptr;
         link = link->NextSiblingElement())
    {
        if (strcmp(link->Value(), "link") == 0)
        {
            for (tinyxml2::XMLElement *collision = link->FirstChildElement();
                 collision != nullptr;
                 collision = collision->NextSiblingElement())
            {
                if (strcmp(collision->Value(), "collision") == 0)
                {
                    for (tinyxml2::XMLElement *geometry = collision->FirstChildElement();
                         geometry != nullptr;
                         geometry = geometry->NextSiblingElement())
                    {
                        if (strcmp(geometry->Value(), "geometry") == 0)
                        {
                            for (tinyxml2::XMLElement *mesh = geometry->FirstChildElement();
                                 mesh != nullptr;
                                 mesh = mesh->NextSiblingElement())
                            {
                                if (strcmp(mesh->Value(), "mesh") == 0)
                                {
                                    boost::filesystem::path mesh_path = mesh->Attribute("filename");

                                    mesh->SetAttribute("filename", mesh_path.filename().c_str());
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    doc.SaveFile(model_urdf_path.c_str());
}

// modify input file
void load_urdf(const char *input, const char *output)
{
    boost::filesystem::path input_file_path = input;
    boost::filesystem::path output_file_path = output;

    urdf::Model model;
    if (!model.initFile(input_file_path.c_str()))
    {
        ROS_ERROR("Couldn't read file in [%s]\n", input);
    }

    meshes_path_string = output_file_path.stem().string() + "/meshes";

    boost::filesystem::create_directories(output_file_path.parent_path() / meshes_path_string);
    boost::filesystem::path meshes_path = output_file_path.parent_path() / meshes_path_string;

    boost::filesystem::path model_urdf_path = meshes_path / input_file_path.filename();
    if (boost::filesystem::exists(model_urdf_path))
    {
        ROS_INFO("File [%s] exists, replace...", model_urdf_path.c_str());
        boost::filesystem::remove(model_urdf_path);
    }
    boost::filesystem::copy_file(input_file_path, model_urdf_path);

    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);
    for (const urdf::LinkSharedPtr &link : links)
    {
        for (const urdf::VisualConstSharedPtr &visual : link->visual_array)
        {
            if (visual->geometry->type == urdf::Geometry::MESH)
            {
                urdf::Mesh &mesh = dynamic_cast<urdf::Mesh &>(*visual->geometry);
                mesh.filename.erase(0, 9);
                boost::filesystem::path mesh_path = mesh.filename;

                std::string file_name = mesh_path.filename().string();
                boost::filesystem::path ros_pkg = mesh_path;
                while (ros_pkg.has_parent_path() && ros_pkg.parent_path().has_parent_path())
                {
                    ros_pkg = ros_pkg.parent_path();
                }
                ros_pkg = ros_pkg.relative_path();
                boost::filesystem::path ros_pkg_path = ros::package::getPath(ros_pkg.string());
                mesh_path = ros_pkg_path.parent_path() / mesh_path.string();

                if (boost::filesystem::exists(meshes_path / file_name))
                {
                    ROS_INFO("File [%s] from [%s] already exists in [%s], ignore", file_name.c_str(), mesh_path.c_str(), meshes_path.c_str());
                }
                else
                {
                    boost::filesystem::copy_file(mesh_path, meshes_path / mesh_path.filename());
                }
            }
        }
        for (const urdf::CollisionSharedPtr &collision : link->collision_array)
        {
            if (collision->geometry->type == urdf::Geometry::MESH)
            {
                urdf::Mesh &mesh = dynamic_cast<urdf::Mesh &>(*collision->geometry);
                mesh.filename.erase(0, 9);
                boost::filesystem::path mesh_path = mesh.filename;
                std::string file_name = mesh_path.filename().string();
                boost::filesystem::path ros_pkg = mesh_path;
                while (ros_pkg.has_parent_path() && ros_pkg.parent_path().has_parent_path())
                {
                    ros_pkg = ros_pkg.parent_path();
                }
                ros_pkg = ros_pkg.relative_path();
                boost::filesystem::path ros_pkg_path = ros::package::getPath(ros_pkg.string());
                mesh_path = ros_pkg_path.parent_path() / mesh_path.string();

                if (boost::filesystem::exists(meshes_path / file_name))
                {
                    ROS_INFO("File [%s] from [%s] already exists in [%s], ignore", file_name.c_str(), mesh_path.c_str(), meshes_path.c_str());
                }
                else
                {
                    boost::filesystem::copy_file(mesh_path, meshes_path / mesh_path.filename());
                }
            }
        }
    }

    add_mujoco_tags(model_urdf_path);

    // load model
    m = mj_loadXML(model_urdf_path.c_str(), 0, error, 1000);
}

// main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mujoco_compile");

    // print help if arguments are missing
    if (argc < 2)
    {
        return finish(helpstring);
    }

    // determine file types
    int type1 = filetype(argv[1]);
    int type2;
    std::string output;
    if (argc == 2)
    {
        type2 = typeXML;
        output = ros::package::getPath("mujoco_sim") + "/model/tmp/robot.xml";
    }
    else
    {
        type2 = filetype(argv[2]);
        output = argv[2];
    }

    // check types
    if (type1 != typeURDF || type2 != typeXML)
    {
        return finish("Illegal combination of file formats");
    }

    // override output file if it exists
    if (boost::filesystem::exists(output))
    {
        boost::filesystem::remove(output);
    }

    load_urdf(argv[1], output.c_str());

    // check error
    if (!m)
    {
        return finish(error);
    }

    // save model
    if (!mj_saveLastXML(output.c_str(), m, error, 1000))
    {
        return finish(error);
    }

    // finalize
    return finish("Done");
}
