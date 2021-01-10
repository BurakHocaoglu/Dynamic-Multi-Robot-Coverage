#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *
from abc import ABC, abstractmethod

class ConstraintBase(ABC):

	def __init__(self, cname):
		self._name = cname

	# def isPointContained(self, point):
	# 	pass

	# def isSimpleConstraintContained(self, constr):
	# 	pass

	# def isConstraintSetContained(self, constr_set):
	# 	raise ItemNotImplementedError("SimpleConstraint.isConstraintSetContained")

class LineConstraint:

	def __init__(self, cname="", normal=None, point=None):
		self._name = cname
		self._normal = normal
		self._point = point

	@property
	def normal(self):
		return self._foo

class ConstraintSet:

	def __init__(self, 
				 constr_name="", 
				 constrs=dict()):
		self.constr_name = constr_name
		self.constraints = constrs

	# def isPointContained(self, point):
	# 	pass

	# def isSimpleConstraintContained(self, constr):
	# 	pass

	# def isConstraintSetContained(self, constr_set):
	# 	pass