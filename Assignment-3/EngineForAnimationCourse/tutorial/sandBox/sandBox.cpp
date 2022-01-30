#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
#include <igl/PI.h>
double calcAngle(Eigen::Vector3d v1, Eigen::Vector3d v2);
void AddAxes(igl::opengl::ViewerData &data, const Eigen::Vector3d &center);
Eigen::Vector3d transform_vec3(Eigen::Matrix4d trans, Eigen::Vector3d vec3);

SandBox::SandBox()
{
}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file " << config << std::endl;
	}
	else
	{
		int count = 0;
		parents.push_back(-1);
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);

			parents.push_back(-1);
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);
			if (count == 0)
			{
				dest_idx = count;
				data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
				data().MyTranslate(Eigen::Vector3d(5, 0, 0), true);
			}
			else
			{
				Eigen::RowVector3d center(0, 0, 0.8);
				// After checking with AlignedBox, the length along the z axis of the zCylinder is 1.6
				if (count == 1)
				{
					data().MyTranslate(-center, true);
					first_link_idx = count;
				}
				else
					data().MyTranslate(-2 * center, true);
				data().SetCenterOfRotation(center);
				data().add_points(center, Eigen::RowVector3d(0, 0, 1));
				AddAxes(data(), -center);
				parents[count + 1] = count;
				link_tip = -center;
				links.push_back(count);
			}
			count++;
		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);

	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
	std::cout << "IK solver: FABRIK" << std::endl;
	std::cout << "Rotation unlimited" << std::endl;
}

void AddAxes(igl::opengl::ViewerData &data, const Eigen::Vector3d &center)
{
	Eigen::Matrix3d colors;
	colors.row(0) << 0, 0, 1;
	colors.row(1) << 0, 1, 0;
	colors.row(2) << 1, 0, 0;

	Eigen::Matrix3d P1;
	P1.row(0) = center + Eigen::Vector3d(1.6, 0, 0);
	P1.row(1) = center + Eigen::Vector3d(0, 1.6, 0);
	P1.row(2) = center + Eigen::Vector3d(0, 0, 1.6);

	Eigen::Matrix3d P2;
	P2.row(0) = center - Eigen::Vector3d(1.6, 0, 0);
	P2.row(1) = center - Eigen::Vector3d(0, 1.6, 0);
	P2.row(2) = center - Eigen::Vector3d(0, 0, 1.6);

	data.add_edges(P1, P2, colors);
}

void SandBox::print_transformations()
{
	Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
	for (int link : links)
	{
		trans *= data_list[link].MakeTransd();
		Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "(", ")");
		std::cout << "Link " << link << " transformation:\n"
				  << trans.format(CleanFmt) << "\n"
				  << std::endl;
	}
}

void SandBox::print_tip_positions()
{
	Eigen::Vector3d tip;
	for (int link : links)
	{
		tip = transform_vec3(CalcParentsTrans(link) * data_list[link].MakeTransd(), link_tip);
		Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "(", ")");
		std::cout << "Link " << link << " tip: " << tip.transpose().format(CleanFmt) << std::endl;
	}
}

void SandBox::print_destination()
{
	Eigen::Vector3d curr_dest = transform_vec3(data_list[dest_idx].MakeTransd(), Eigen::Vector3d::Zero());
	Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "(", ")");
	std::cout << "Destination: " << curr_dest.transpose().format(CleanFmt) << std::endl;
}

SandBox::~SandBox()
{
}

void SandBox::Animate()
{
	if (isActive)
	{
		if (FABRIK)
		{
			FABRIK_iteration();
		}
		else
		{

			CCD_iteration();
		}
	}
}

void SandBox::CCD_iteration()
{
	Eigen::Vector3d dest = data_list[dest_idx].GetTranslation();
	Eigen::Vector3d base = (data_list[first_link_idx].MakeTransd() * Eigen::Vector4d(0, 0, 0.8, 1)).head(3);
	Eigen::Vector3d last_tip, curr_tip, v1, v2, perp;
	Eigen::Matrix3d curr_rot;
	double angle;
	int last_idx = data_list.size() - 1;
	if ((base - dest).norm() > 1.6 * links.size())
	{
		std::cout << "cannot reach" << std::endl;
		return;
	}
	for (int i = last_idx; i >= first_link_idx; i--)
	{
		last_tip = (CalcParentsTrans(last_idx) * data_list[last_idx].MakeTransd() * Eigen::Vector4d(0, 0, -0.8, 1)).head(3);
		curr_tip = (CalcParentsTrans(i) * Eigen::Vector4d(0, 0, -0.8, 1)).head(3);
		if ((last_tip - dest).norm() < 0.1)
		{
			std::cout << "distance: " << (last_tip - dest).norm() << std::endl;
			break;
		}

		v1 = last_tip - curr_tip;
		v2 = dest - curr_tip;
		
		if (v2 == Eigen::Vector3d(0,0,0))
			v2 = Eigen::Vector3d(0,0,0.01);
		
		perp = v1.cross(v2).normalized();

		if (v1.normalized() == v2.normalized() || v1.normalized() == -v2.normalized())
		{
			if(v1.x() != 0 || v1.y() != 0)
				perp = Eigen::Vector3d(v1.y(), -v1.x(), 0.);
			else
				perp = Eigen::Vector3d(1., 0, 0);
		}

		angle = calcAngle(v1, v2) / 10.0;

		if (isLimited)
		{
			Eigen::Vector3d parent_z_axis = -((data_list[i].GetRotation() * Eigen::AngleAxisd(angle, perp).matrix()).transpose() * Eigen::Vector3d(0, 0, 1));
			double alpha = calcAngle(parent_z_axis, Eigen::Vector3d(0, 0, 1));
			if (alpha < (M_PI / 6))
			{
				angle = angle > 0.0 ? angle - ((M_PI / 6) - alpha) : angle + ((M_PI / 6) - alpha);
			}
		}

		curr_rot = CalcParentsTrans(i).block<3, 3>(0, 0) * data_list[i].GetRotation();
		data_list[i].MyRotate(curr_rot.transpose() * perp, angle);
	}
}

void SandBox::FABRIK_iteration()
{
	Eigen::Vector3d t = data_list[dest_idx].GetTranslation();
	double d = 1.6;
	int n = links.size() + 1;
	std::vector<Eigen::Matrix3d> rotations(n - 1);
	std::vector<Eigen::Vector3d> p(n);
	std::vector<Eigen::Vector3d> tips(n);
	std::vector<double> r(n - 1);
	std::vector<double> lam(n - 1);
	Eigen::Vector3d b, v1, v2, perp;
	double angle, dist;
	for (int i = 0; i < n - 1; i++)
	{
		p[i] = (CalcParentsTrans(links[i]) * data_list[links[i]].MakeTransd() * Eigen::Vector4d(0, 0, 0.8, 1)).head(3);
	}
	p[n - 1] = (CalcParentsTrans(links[n - 2]) * data_list[links[n - 2]].MakeTransd() * Eigen::Vector4d(0, 0, -0.8, 1)).head(3);
	tips = p;
	dist = (p[0] - t).norm();
	if (dist > 1.6 * (n - 1))
	{
		std::cout << "cannot reach" << std::endl;
		return;
	}
	b = p[0];
	if ((p[n - 1] - t).norm() < 0.1)
	{
		std::cout << "distance: " << (p[n - 1] - t).norm() << std::endl;
		return;
	}
	// calculate the points
	// forward
	p[n - 1] = t;
	for (int i = n - 2; i >= 0; i--)
	{
		r[i] = (p[i + 1] - p[i]).norm();
		lam[i] = d / r[i];
		p[i] = (1 - lam[i]) * p[i + 1] + lam[i] * p[i];
		if (isLimited && i != n - 2)
		{
			Eigen::Vector3d next_joint = p[i + 2] - p[i + 1];
			Eigen::Vector3d curr_joint = p[i] - p[i + 1];
			double alpha = calcAngle(next_joint, curr_joint);
			if (alpha < (M_PI / 6))
			{
				if (next_joint == Eigen::Vector3d(0,0,0))
					next_joint = Eigen::Vector3d(0,0,0.01);
				perp = next_joint.cross(curr_joint);
				if (curr_joint.normalized() == next_joint.normalized() || curr_joint.normalized() == -next_joint.normalized())
				{
					if(curr_joint.x() != 0 || curr_joint.y() != 0)
						perp = Eigen::Vector3d(curr_joint.y(), -curr_joint.x(), 0.);
					else
						perp = Eigen::Vector3d(1., 0, 0);
				}
				p[i] = p[i + 1] + Eigen::AngleAxisd(M_PI / 6, perp.normalized()) * next_joint;
			}
		}
	}
	// backward
	p[0] = b;
	for (int i = 0; i < n - 1; i++)
	{
		r[i] = (p[i + 1] - p[i]).norm();
		lam[i] = d / r[i];
		p[i + 1] = (1 - lam[i]) * p[i] + lam[i] * p[i + 1];
	}

	// apply the rotations
	Eigen::Matrix3d curr_rot = Eigen::Matrix3d::Identity();
	for (int i = 0; i < n - 1; i++)
	{
		v1 = tips[i + 1] - tips[i];
		v2 = p[i + 1] - tips[i];

		if (v2 == Eigen::Vector3d(0,0,0))
			v2 = Eigen::Vector3d(0,0,0.01);

		perp = v1.cross(v2).normalized();

		if (v1.normalized() == v2.normalized() || v1.normalized() == -v2.normalized())
		{
			if(v1.x() != 0 || v1.y() != 0)
				perp = Eigen::Vector3d(v1.y(), -v1.x(), 0.);
			else
				perp = Eigen::Vector3d(1., 0, 0);
		}

		angle = calcAngle(v1, v2) / 10.0;
		curr_rot = CalcParentsTrans(links[i]).block<3, 3>(0, 0) * data_list[links[i]].GetRotation();

		data_list[links[i]].MyRotate(curr_rot.transpose() * perp, angle);

		for (int j = 0; j < n - 1; j++)
		{
			tips[j] = transform_vec3(CalcParentsTrans(links[j]) * data_list[links[j]].MakeTransd(), Eigen::Vector3d(0, 0, 0.8));
		}
		tips[n - 1] = transform_vec3(CalcParentsTrans(links[n - 2]) * data_list[links[n - 2]].MakeTransd(), Eigen::Vector3d(0, 0, -0.8));
	}
}

Eigen::Vector3d transform_vec3(Eigen::Matrix4d trans, Eigen::Vector3d vec3)
{
	Eigen::Vector4d vec4(vec3.x(), vec3.y(), vec3.z(), 1);
	return (trans * vec4).head(3);
}