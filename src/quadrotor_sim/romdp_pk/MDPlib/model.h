#ifndef _MDP_MODEL_HEADER__
#define _MDP_MODEL_HEADER__

#include <eigen3/Eigen/Dense>
namespace MDP
{
	class Model
	{
		public:
			Model();
			~Model();

			void setDiscount(const double e_);
			const double getDiscount();

			void set_action_num(const int M_);
			void set_state_num(const int N_);
			const int get_action_num();
			const int get_state_num();

			void set_env_col(const int col_);
			void set_env_row(const int row_);

			const int get_env_col();
			const int get_env_row();

		private:
			int col, row;
			int N;  // dimension of states
			int M;  // dimension of actions
			double gamma; // discount
	};
}


#endif
