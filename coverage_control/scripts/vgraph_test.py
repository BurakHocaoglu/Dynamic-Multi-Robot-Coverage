#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *

if __name__ == "__main__":
	g = VGraph()

	# g.add_edge("d_ef", "d_eg", VEdge("H", np.array([  2.9,   9.3], dtype=float)))
	# g.add_edge("d_ef", "b_ab", VEdge("I", np.array([ -7.5,  30.0], dtype=float)))
	# g.add_edge("d_eg", "b_dc", VEdge("L", np.array([ -5.0, -30.0], dtype=float)))
	# g.add_edge("b_ab", "b_da", VEdge("A", np.array([-30.0,  30.0], dtype=float)))
	# g.add_edge("b_dc", "b_da", VEdge("D", np.array([-30.0, -30.0], dtype=float)))

	g.add_edge("d_kl", "d_km", VEdge("P", np.array([  22.5,   32.5], dtype=float)))
	g.add_edge("d_kl", "b_dh", VEdge("O", np.array([ 22.5,  53.9], dtype=float)))
	g.add_edge("d_km", "b_gc", VEdge("N", np.array([ 35.7, 25.9], dtype=float)))
	g.add_edge("b_dh", "b_hc", VEdge("H", np.array([27.6,  38.0], dtype=float)))
	g.add_edge("b_gc", "b_hc", VEdge("C", np.array([52.4, 38.0], dtype=float)))

	cell = g.traverse()

	print("Traversed Cell:")
	for v in cell:
		print("{}".format(v))
