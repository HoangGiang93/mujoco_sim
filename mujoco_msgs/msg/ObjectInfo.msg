uint8 CUBE=0
uint8 SPHERE=1
uint8 CYLINDER=2
uint8 MESH=3

string name
int32 type
bool movable
geometry_msgs/Vector3 size
std_msgs/ColorRGBA rgba
geometry_msgs/Inertia inertial
string mesh