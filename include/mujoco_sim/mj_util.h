#pragma once

#include <algorithm>
#include <chrono>
#include <mujoco/mujoco.h>
#include <sstream>
#include <tinyxml2.h>

#ifndef TIMEOUT
#define TIMEOUT 500000 //microsecond 
#endif

template <typename T>
static void do_each_child_element(tinyxml2::XMLElement *element, const char *child_elment_type, T &arg, std::function<void(tinyxml2::XMLElement *, T &)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement(child_elment_type);
			 child_element != nullptr;
			 child_element = child_element->NextSiblingElement(child_elment_type))
	{
		function(child_element, arg);
		if (strcmp(child_elment_type, "body") == 0)
		{
			do_each_child_element(child_element, child_elment_type, arg, function);
		}
	}
}

template <typename T>
static void do_each_child_element(tinyxml2::XMLElement *element, const char *child_elment_type, T arg, std::function<void(tinyxml2::XMLElement *, T)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement(child_elment_type);
			 child_element != nullptr;
			 child_element = child_element->NextSiblingElement(child_elment_type))
	{
		function(child_element, arg);
		if (strcmp(child_elment_type, "body") == 0)
		{
			do_each_child_element(child_element, child_elment_type, arg, function);
		}
	}
}

static void do_each_child_element(tinyxml2::XMLElement *element, const char *child_elment_type, std::function<void(tinyxml2::XMLElement *)> function)
{
	for (tinyxml2::XMLElement *child_element = element->FirstChildElement(child_elment_type);
			 child_element != nullptr;
			 child_element = child_element->NextSiblingElement(child_elment_type))
	{
		function(child_element);
		if (strcmp(child_elment_type, "body") == 0)
		{
			do_each_child_element(child_element, child_elment_type, function);
		}
	}
}

template <typename T>
static bool manage_XML(T &arg, const char *path, std::function<bool(T &arg, const char *)> func)
{
	auto start = std::chrono::high_resolution_clock::now();
	bool success = func(arg, path);
	auto stop = std::chrono::high_resolution_clock::now();

	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	std::ostringstream oss;
	if (duration.count() > TIMEOUT)
	{
		oss << "Load" << path << " takes " << duration.count() / 1000000.0 << " seconds";
		mju_warning(oss.str().c_str());
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
	{ return doc.LoadFile(path) != tinyxml2::XML_SUCCESS; };
	return manage_XML(doc, path, load_XML_cb);
}

static bool save_XML(mjModel *&m, const char *path)
{
	std::function<bool(mjModel *&, const char *)> save_XML_cb = [](mjModel *&m, const char *path)
	{
		char error[100] = "Could not save binary model";
		mj_saveLastXML(path, m, error, 100);
		return m != nullptr; };
	return manage_XML(m, path, save_XML_cb);
}

static bool save_XML(tinyxml2::XMLDocument &doc, const char *path)
{
	std::function<bool(tinyxml2::XMLDocument &, const char *)> save_XML_cb = [](tinyxml2::XMLDocument &doc, const char *path)
	{ return doc.SaveFile(path) != tinyxml2::XML_SUCCESS; };
	return manage_XML(doc, path, save_XML_cb);
}