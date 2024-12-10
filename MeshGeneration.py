#!/usr/bin/env python3
# Created: Dec, 09, 2024 14:59:42 by Wataru Fukuda

import watfio
import numpy
import os
import nurbs.UniformBspline

# DEBUG = True
DEBUG = False

class MeshGeneration:
  def __init__(self, space, nes, orders):
    self.space = space
    self.nes = nes
    self.orders = [orders for _ in range(len(nes))]
    self.nsd = len(nes)
    self.npd = len(nes)
    self.nens = []
    self.nen = 1
    for i in range(self.npd):
      self.nens.append(self.orders[i]+1)
      self.nen *= self.nens[-1]
    self.ne = 1
    for i in self.nes:
      self.ne *= i
  def genXYZ(self):
    xyzs = []
    for i in range(len(self.nes)):
      x_ne = self.nes[i]
      order = self.orders[i]
      xyz = nurbs.UniformBspline.UniformBspline.genControlPoints(x_ne, order)
      xpo = self.space[i][0], self.space[i][1]
      for i in range(len(xyz)):
        xyz[i] = xyz[i]*(xpo[1]-xpo[0])+xpo[0]
      xyzs.append(xyz)
    return xyzs
  def toIndexArray(self, index, total):   # each element position from left bottom
    a = []
    for t in total:
      a.append(index % t)
      index //= t
    return a
  def toIndex(self, a, total):
    s = 0
    a_ = a[::-1]
    t_ = total[::-1]
    for a,t in zip(a_,t_):
      s *= t
      s += a
    return s
  def swapIen(self, mien):
    for i in range(len(mien)):
      if(i % 4 == 2):
        mien_ = mien[i]
        mien[i] = mien[i+1]
        mien[i+1] = mien_
    return mien
  def genNSD(self):
    return self.nsd
  def genNPD(self):
    return self.npd
  def genNN(self):
    nn = 1
    for i in self.nes:
      nn *= i + 1
    return nn
  def genNE(self):
    return self.ne
  def genNEN(self):
    return self.nen
  def genMXYZ(self):
    xyz = self.genXYZ()
    nns = []
    for i in range(len(self.nes)):
      nns.append(len(xyz[i]))
    nns_ = nns[::-1]
    nns_.append(len(self.nes))
    cxyz = numpy.empty(nns_, dtype=">f8")
    if(self.npd==2):
      for i in range(len(xyz[0])):
        cxyz[:,i,0] = xyz[0][i]
      for i in range(len(xyz[1])):
        cxyz[i,:,1] = xyz[1][i]
    elif(self.npd==3):
      for i in range(len(xyz[0])):
         cxyz[:,:,i,0] = xyz[0][i]
      for i in range(len(xyz[1])):
        cxyz[:,i,:,1] = xyz[1][i]
      for i in range(len(xyz[2])):
        cxyz[i,:,:,2] = xyz[2][i]
    else:
      print("npd=1 error")
      exit(1)
    nn = self.genNN()
    cxyz = cxyz.reshape(nn,self.npd)
    pxyz = numpy.zeros((nn,self.nsd), dtype=">f8")
    pxyz[:,0:self.npd] = cxyz
    mxyz = numpy.array(pxyz,dtype=">f8")
    return mxyz
  def genMIEN(self):
    xyz = self.genXYZ()
    nns = []
    for i in range(len(self.nes)):
      nns.append(len(xyz[i]))
    nns_ = nns[::-1]
    nns_.append(len(self.nes))
    mien = []
    for j in range(self.ne):
      ele_position = numpy.array(self.toIndexArray(j, self.nes))   # each element position from left bottom
      for k in range(self.nen):
        ele_point_position = numpy.array(self.toIndexArray(k, self.nens))   # each point position within one element from left bottom
        point_position = ele_position + ele_point_position   # each point position in whole mesh
        mien_ = self.toIndex(point_position, nns)
        mien.append(mien_)
    mien = self.swapIen(mien)
    mien = numpy.array(mien,dtype=">i4")
    return mien
  def genMRNG(self):
    nes_= self.nes[::-1]  # reverse order of nes
    nes_.extend((self.npd,2))
    mrng=numpy.zeros(nes_, dtype=">i4")
    mrng -= 1
    if(self.npd==2):  # todo: needed to be changed
      mrng[ :,  0, 0, 0] = 1
      mrng[ :, -1, 0, 1] = 2
      mrng[ 0,  :, 1, 0] = 3
      mrng[-1,  :, 1, 1] = 4
    elif(self.npd==3):
      mrng[ :, :, 0, 0, 0] = 1
      mrng[ :, :,-1, 0, 1] = 2
      mrng[ :, 0, :, 1, 0] = 3
      mrng[ :,-1, :, 1, 1] = 4
      mrng[ 0, :, :, 2, 0] = 5
      mrng[-1, :, :, 2, 1] = 6
    mrng = numpy.array(mrng,dtype=">i4")
    return mrng

