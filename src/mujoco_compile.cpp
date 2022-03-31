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

#include <cctype>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <experimental/filesystem>
#include <ros/package.h>
#include <tinyxml2.h>
#include <urdf/model.h>

#include "mujoco.h"

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

// modify input file
void load_urdf(const char *input, const char *output)
{
    std::experimental::filesystem::path input_file_path = input;
    std::experimental::filesystem::path output_file_path = output;

    ros::NodeHandle n;

    urdf::Model model;
    if (!model.initFile(input_file_path))
    {
        ROS_ERROR("Couldn't read file in [%s]\n", input);
    }

    meshes_path_string = output_file_path.stem().string() + "/meshes";
    std::experimental::filesystem::create_directories(output_file_path.parent_path() / meshes_path_string);
    std::experimental::filesystem::path meshes_path = output_file_path.parent_path() / meshes_path_string;
    copy(input_file_path, meshes_path);

    std::vector<urdf::LinkSharedPtr> links;
    model.getLinks(links);
    for (const urdf::LinkSharedPtr &link : links)
    {
        for (const urdf::CollisionSharedPtr &collision : link->collision_array)
        {
            if (collision->geometry->type == urdf::Geometry::MESH)
            {
                urdf::Mesh &mesh = dynamic_cast<urdf::Mesh &>(*collision->geometry);
                mesh.filename.erase(0, 9);
                std::experimental::filesystem::path mesh_path = mesh.filename.c_str();
                std::string file_name = mesh_path.filename();
                std::experimental::filesystem::path ros_pkg = mesh_path;
                while (ros_pkg.has_parent_path() && ros_pkg.parent_path().has_parent_path())
                {
                    ros_pkg = ros_pkg.parent_path();
                }
                ros_pkg = ros_pkg.relative_path();
                std::experimental::filesystem::path ros_pkg_path = ros::package::getPath(ros_pkg);
                mesh_path = ros_pkg_path.parent_path() / mesh_path.string();
                if (std::experimental::filesystem::exists(meshes_path / file_name))
                {
                    ROS_WARN("File [%s] from [%s] already exists in [%s], ignore", file_name.c_str(), mesh_path.c_str(), meshes_path.c_str());
                }
                else
                {
                    copy(mesh_path, meshes_path);
                }
            }
        }
    }

    std::experimental::filesystem::path model_urdf_path = meshes_path / input_file_path.filename();

    // load model
    m = mj_loadXML(model_urdf_path.c_str(), 0, error, 1000);

    std::experimental::filesystem::remove(model_urdf_path);
}

// modify output file
void modify_xml(const char *output)
{
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(output) != tinyxml2::XML_SUCCESS)
    {
        mju_error_s("Couldn't read file in [%s]\n", output);
    }
    tinyxml2::XMLElement *asset_element = doc.FirstChildElement()->FirstChildElement();
    while (asset_element != nullptr)
    {
        if (strcmp(asset_element->Value(), "asset") == 0)
        {
            tinyxml2::XMLElement *mesh_element = asset_element->FirstChildElement();
            while (mesh_element != nullptr)
            {
                std::string file_name = mesh_element->Attribute("file");
                mesh_element->SetAttribute("file", (meshes_path_string + "/" + file_name).c_str());
                mesh_element = mesh_element->NextSiblingElement();
            }
        }
        asset_element = asset_element->NextSiblingElement();
    }
    doc.SaveFile(output);
}

// main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mujoco_compile");

    // print help if arguments are missing
    if (argc != 3)
    {
        return finish(helpstring);
    }

    // determine file types
    int type1 = filetype(argv[1]);
    int type2 = filetype(argv[2]);

    // check types
    if (type1 != typeURDF || type2 != typeXML)
    {
        return finish("Illegal combination of file formats");
    }

    // make sure output file does not exist
    std::FILE *fp = std::fopen(argv[2], "r");
    if (fp)
    {
        std::fclose(fp);
        return finish("Output file already exists");
    }

    load_urdf(argv[1], argv[2]);

    // check error
    if (!m)
    {
        return finish(error);
    }

    // save model
    if (!mj_saveLastXML(argv[2], m, error, 1000))
    {
        return finish(error);
    }

    modify_xml(argv[2]);

    // finalize
    return finish("Done");
}
