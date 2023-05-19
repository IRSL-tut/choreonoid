#include <pugixml.hpp>

#include <iostream>

int main(int argc, char **argv)
{
    pugi::xml_document doc;

    pugi::xml_node root = doc.append_child("robot");

    {
        root.append_attribute("name") = "robot_name";
        root.append_attribute("xmlns:xi") = "http://www.w3.org/2001/Xinclude";
        root.append_attribute("xmlns:gazebo"    ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#gz";
        root.append_attribute("xmlns:model"     ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#model";
        root.append_attribute("xmlns:sensor"    ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor";
        root.append_attribute("xmlns:body"      ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#body";
        root.append_attribute("xmlns:geom"      ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#geom";
        root.append_attribute("xmlns:joint"     ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#joint";
        root.append_attribute("xmlns:interface" ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#interface";
        root.append_attribute("xmlns:rendering" ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering";
        root.append_attribute("xmlns:renderable") = "http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable";
        root.append_attribute("xmlns:controller") = "http://playerstage.sourceforge.net/gazebo/xmlschema/#controller";
        root.append_attribute("xmlns:physics"   ) = "http://playerstage.sourceforge.net/gazebo/xmlschema/#physics";
        root.append_attribute("xmlns:xacro"     ) = "http://www.ros.org/wiki/xacro";
    }

    {// add trans xacro
        pugi::xml_node xac = root.append_child("xacro:macro");

        xac.append_attribute("name") = "gazobo_transmission";
        xac.append_attribute("params") = "joint_name";

        pugi::xml_node trans = xac.append_child("transmission");
        trans.append_attribute("name") = "${joint_name}_trans";

        trans.append_child("type")
            .append_child(pugi::node_pcdata)
            .set_value("transmission_interface/SimpleTransmission");

        pugi::xml_node t_joint = trans.append_child("joint");
        t_joint.append_attribute("name") = "${joint_name}";
        t_joint.append_child("hardwareInterface")
            .append_child(pugi::node_pcdata)
            .set_value("hardware_interface/PositionJointInterface</hardwareInterface");

        pugi::xml_node t_act   = trans.append_child("actuator");
        t_act.append_attribute("name") = "${joint_name}_motor";
        t_act.append_child("hardwareInterface")
            .append_child(pugi::node_pcdata)
            .set_value("hardware_interface/PositionJointInterface</hardwareInterface");
    }

    {
        pugi::xml_node comment = root.append_child(pugi::node_comment);
        comment.set_value("hogemoge");
    }

    {
        pugi::xml_node nd = root.append_child("desc");
        nd.append_attribute("name") = "at_name";
        nd.append_attribute("val(int)") = 1;
        nd.append_attribute("val(float)") = 1.234567890123456789;
    }

    doc.save_file("/tmp/hoge.urdf");

    return 0;
}
