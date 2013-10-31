/*
 * PointFusion.h
 *
 *  Created on: 5 Aug 2013
 *      Author: cm00215
 */

#ifndef POINTFUSION_H_
#define POINTFUSION_H_

class PointFusion {
	public:
		PointFusion();
		virtual ~PointFusion();

		void render_model_pov(void);
		void generate_index_map(void);
		void do_registration(void);
		void integrate_measurements(void);
		void save_model(void);

	private:

};

#endif /* POINTFUSION_H_ */
