#include "model.h"
#include <eigen3/Eigen/Dense>

namespace MDP
{
	Model::Model()
	{

	}

	Model::~Model()
	{

	}

	void Model::set_env_row(const int row_)
	{
		row = row_;
	}
	const int Model::get_env_row()
	{
		return row;
	}

	void Model::set_env_col(const int col_)
	{
		col = col_;
	}
	const int Model::get_env_col()
	{
		return col;
	}

	void Model::set_state_num(const int M_)
	{
		M = M_;
	}
	const int Model::get_state_num()
	{
		return M;
	}

	void Model::set_action_num(const int N_)
	{
		N = N_;
	}
	const int Model::get_action_num()
	{
		return N;
	}
	
	void Model::setDiscount(const double e_)
	{
		gamma = e_;
	}
	const double Model::getDiscount()
	{
		return gamma;
	}



}
