#include <algorithm>
#include <chrono>
#include <mujoco/mujoco.h>
#include <sstream>
#include <tinyxml2.h>
#include <functional>
#include <ros/ros.h>

#ifndef TIMEOUT
#define TIMEOUT 500000 // microsecond
#endif

template <typename T>
static void do_each_child_element(tinyxml2::XMLElement *element, const char *child_element_type, T &arg, std::function<void(tinyxml2::XMLElement *, T &)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement(child_element_type);
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement(child_element_type))
	{
		function(child_element, arg);
	}

	for (tinyxml2::XMLElement *child_element = element->FirstChildElement("body");
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement("body"))
	{
		do_each_child_element(child_element, child_element_type, arg, function);
	}
}

template <typename T>
static void do_each_child_element(tinyxml2::XMLElement *element, const char *child_element_type, T arg, std::function<void(tinyxml2::XMLElement *, T)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement(child_element_type);
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement(child_element_type))
	{
		function(child_element, arg);
	}
	
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement("body");
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement("body"))
	{
		do_each_child_element(child_element, child_element_type, arg, function);
	}
}

template <typename T>
static void do_each_child_element(tinyxml2::XMLElement *element, T &arg, std::function<void(tinyxml2::XMLElement *, T &)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement("body");
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement("body"))
	{
		function(child_element, arg);
		do_each_child_element(child_element, arg, function);
	}
}

template <typename T>
static void do_each_child_element(tinyxml2::XMLElement *element, T arg, std::function<void(tinyxml2::XMLElement *, T)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement("body");
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement("body"))
	{
		function(child_element, arg);
		do_each_child_element(child_element, arg, function);
	}
}

template <typename T>
static void do_each_child_element(tinyxml2::XMLElement *element, T &arg, std::function<void(tinyxml2::XMLElement *)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement("body");
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement("body"))
	{
		function(child_element);
		do_each_child_element(child_element, arg, function);
	}
}

template <typename T>
static void do_each_child_element(tinyxml2::XMLElement *element, T arg, std::function<void(tinyxml2::XMLElement *)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement("body");
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement("body"))
	{
		function(child_element);
		do_each_child_element(child_element, arg, function);
	}
}

static void do_each_child_element(tinyxml2::XMLElement *element, const char *child_element_type, std::function<void(tinyxml2::XMLElement *)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement(child_element_type);
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement(child_element_type))
	{
		function(child_element);
	}
	
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement("body");
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement("body"))
	{
		do_each_child_element(child_element, child_element_type, function);
	}
}

static void do_each_child_element(tinyxml2::XMLElement *element, std::function<void(tinyxml2::XMLElement *)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement("body");
		 child_element != nullptr;
		 child_element = child_element->NextSiblingElement("body"))
	{
		function(child_element);
		do_each_child_element(child_element, function);
	}
}

static void do_each_child_body_id(mjModel *&m, int body_id, std::function<void(int i)> function)
{
	for (int child_body_id = 0; child_body_id < m->nbody; child_body_id++)
	{
		if (m->body_parentid[child_body_id] == body_id)
		{
			function(child_body_id);
			do_each_child_body_id(m, child_body_id, function);
		}
	}
}

static void do_each_object_type(tinyxml2::XMLElement *element, const mjtObj type, std::function<void(tinyxml2::XMLElement *, const mjtObj)> function)
{
	switch (type)
	{
	case mjtObj::mjOBJ_MESH:
		do_each_child_element(element, "mesh", type, function);
		break;

	case mjtObj::mjOBJ_BODY:
		do_each_child_element(element, "body", type, function);
		break;

	case mjtObj::mjOBJ_JOINT:
		do_each_child_element(element, "joint", type, function);
		break;

	case mjtObj::mjOBJ_GEOM:
		do_each_child_element(element, "geom", type, function);
		break;

	default:
		break;
	}
};

template <typename T>
static bool manage_XML(T &arg, const char *path, std::function<bool(T &arg, const char *)> func)
{
	auto start = std::chrono::high_resolution_clock::now();

	bool success = func(arg, path);
	std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
	while (!success && duration.count() / 1000000.0 < 1.0)
	{
		success = func(arg, path);
		duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start);
	}

	std::ostringstream oss;
	if (duration.count() > TIMEOUT)
	{
		oss << "Load" << path << " takes " << duration.count() / 1000000.0 << " seconds";
		mju_warning("%s", oss.str().c_str());
	}

	return success;
}

static bool load_XML(mjModel *&m, const char *path)
{
	std::function<bool(mjModel *&, const char *)> load_XML_cb = [](mjModel *&m, const char *path)
	{
		char error[100] = "Could not load binary model";
		m = mj_loadXML(path, 0, error, 100);
		return m != nullptr; };
	return manage_XML(m, path, load_XML_cb);
}

static bool load_XML(tinyxml2::XMLDocument &doc, const char *path)
{
	std::function<bool(tinyxml2::XMLDocument &, const char *)> load_XML_cb = [](tinyxml2::XMLDocument &doc, const char *path)
	{ return doc.LoadFile(path) == tinyxml2::XML_SUCCESS; };
	return manage_XML(doc, path, load_XML_cb);
}

static bool save_XML(mjModel *&m, const char *path)
{
	std::function<bool(mjModel *&, const char *)> save_XML_cb = [](mjModel *&m, const char *path)
	{
		char error[100] = "Could not save binary model";
		return mj_saveLastXML(path, m, error, 100) == 1; };
	return manage_XML(m, path, save_XML_cb);
}

static bool save_XML(tinyxml2::XMLDocument &doc, const char *path)
{
	std::function<bool(tinyxml2::XMLDocument &, const char *)> save_XML_cb = [](tinyxml2::XMLDocument &doc, const char *path)
	{ return doc.SaveFile(path) == tinyxml2::XML_SUCCESS; };
	return manage_XML(doc, path, save_XML_cb);
}