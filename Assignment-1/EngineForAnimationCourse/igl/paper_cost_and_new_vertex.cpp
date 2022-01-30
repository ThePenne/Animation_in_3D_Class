#include "paper_cost_and_new_vertex.h"

Eigen::Matrix4d calc_Kp(const Eigen::VectorXd n, const Eigen::VectorXd& v)
{
	double a = n.x(), b = n.y(), c = n.z();
	double d = n.dot(v);
	Eigen::Vector4d p(a, b, c, d);
	Eigen::Matrix4d Kp = p * p.transpose();
	return Kp;
}

Eigen::VectorXd calc_face_normal(int f, const Eigen::MatrixXi& F, const Eigen::MatrixXd& V)
{
	Eigen::Vector3d v1, v2, v3;
	v1 = V.row(F(f, 0));
	v2 = V.row(F(f, 1));
	v3 = V.row(F(f, 2));
	return ((v2 - v1).cross(v2 - v3)).normalized();
}

Eigen::Matrix4d calc_Qv(int e, int v, const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& F,
	const Eigen::MatrixXi& E,
	const Eigen::VectorXi& EMAP,
	const Eigen::MatrixXi& EF)
{
	Eigen::Matrix4d Qv = Eigen::Matrix4d::Zero();
	const int first_f = EF(e, 0);
	int f = first_f;
	int next_e = 0;
	do {

		const Eigen::VectorXd n = calc_face_normal(f, F, V);
		Qv += calc_Kp(n, V.row(v));
		for (int i = 0; i < 3; i++)
		{
			next_e = EMAP(f + i * F.rows());
			if (next_e != e && (E(next_e, 0) == v || E(next_e, 1) == v))
				break;
		}
		e = next_e;
		f = EF(e, 0) != f ? EF(e, 0) : EF(e, 1);
	} while (f != first_f);
	return Qv;
}

IGL_INLINE void igl::paper_cost_and_new_vertex(
	const int e,
	const Eigen::MatrixXd& V,
	const Eigen::MatrixXi& F,
	const Eigen::MatrixXi& E,
	const Eigen::VectorXi& EMAP,
	const Eigen::MatrixXi& EF,
	const Eigen::MatrixXi& EI,
	double& cost,
	Eigen::RowVectorXd& p)
{

	Eigen::Matrix4d Qv1 = calc_Qv(e, E(e, 0), V, F, E, EMAP, EF);

	Eigen::Matrix4d Qv2 = calc_Qv(e, E(e, 1), V, F, E, EMAP, EF);

	Eigen::Matrix4d Qe = Qv1 + Qv2;
	Eigen::Vector4d y(0, 0, 0, 1);
	Eigen::Matrix4d Q_opt;
	Q_opt.row(0) << Qe.row(0);
	Q_opt.row(1) << Qe(0, 1), Qe(1, 1), Qe(1, 2), Qe(1, 3);
	Q_opt.row(2) << Qe(0, 2), Qe(2, 1), Qe(2, 2), Qe(2, 3);
	Q_opt.row(3) << 0, 0, 0, 1;

	Eigen::Vector4d v_opt = - Q_opt.colPivHouseholderQr().solve(y);
	p << v_opt.x(), v_opt.y(), v_opt.z();
	cost = v_opt.transpose() * Qe * v_opt;
}