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

	void render_model_pov();
	void generate_index_map();
	void do_registration();
	void integrate_measurements();
	void save_model();

private:

};

#endif /* POINTFUSION_H_ */
