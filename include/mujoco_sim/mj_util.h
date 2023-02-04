#include <algorithm>
#include <tinyxml2.h>

template <typename T>
static void do_each_child_element(tinyxml2::XMLElement *element, const char *child_elment_type, const T &arg, std::function<void(tinyxml2::XMLElement *, const T &)> function)
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
static void do_each_child_element(tinyxml2::XMLElement *element, const char *child_elment_type, const T arg, std::function<void(tinyxml2::XMLElement *, const T)> function)
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