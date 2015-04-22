import numpy as np
import math
import sys

import math

def norm2(x): return x.dot(x)
def norm(x): return math.sqrt(norm2(x))

def basis(i, n):
    res = np.zeros(n)
    res[i] = 1
    return res


class Rigid3(np.ndarray):

    # velocity/forces
    class Deriv(np.ndarray):
        def __new__(cls, *args):
            return np.ndarray.__new__(cls, 6)

        @property
        def linear(self):
            return self[:3]

        @linear.setter
        def linear(self, value):
            self[:3] = value


        @property
        def angular(self):
            return self[3:]

        @angular.setter
        def angular(self, value):
            self[3:] = value

        @staticmethod
        def array(size):
            return np.zeros( (size, 6) ).view(dtype = Rigid3.Deriv )


        @staticmethod
        def view( array ):
            if len(array) % 6 > 0: raise Exception('bad array size')
            return array.reshape( (len(array) / 6, 6 ) ).view(dtype = Rigid3.Deriv )
        
    @staticmethod
    def array(size):
        res = np.zeros( (size, 7) ).view( dtype = Rigid3 )
        for ri in res:
            ri.orient.real = 1

        return res
    
    @property
    def center(self):
        return self[:3]

    @center.setter
    def center(self, value):
        self[:3] = value

    @property
    def orient(self):
        return self[3:].view(Quaternion)

    @orient.setter
    def orient(self, value):
        self[3:] = value

    def __new__(cls, *args):
        return np.ndarray.__new__(cls, 7)
        
    def __init__(self):
        self[-1] = 1
        self[:6] = 0

    def inv(self):
        res = Rigid3()
        res.orient = self.orient.inv()
        res.center = -res.orient(self.center)
        return res

    def __mul__(self, other):
        res = Rigid3()

        res.orient = self.orient * other.orient
        res.center = self.center + self.orient(other.center)
        
        return res

    def __call__(self, x):
        '''applies rigid transform to vector x'''
        return self.center + self.orient(x)


    def dump(self):
        return self.tolist()

    def load(self, x):
        self[:] = x
    
    # def dump(self):
    #     return self.__dict__

    # def load(self, x):
    #     self.__dict__ = x
    


    
class Quaternion(np.ndarray):

    class Deriv(np.ndarray):
        def __new__(cls, *args):
            return np.ndarray.__new__(cls, 3)

        @staticmethod
        def array(size):
            return np.zeros( (size, 3) ).view(dtype = Deriv)



    rad = math.pi / 180.0
    deg = 180.0 / math.pi
    
    def __new__(cls, *args):
        return np.ndarray.__new__(cls, 4)
        
    def __init__(self):
        '''identity quaternion'''
        self.real = 1
        self.imag = 0
        
    def inv(self):
        '''inverse'''
        return self.conj() / self.dot(self)

    def dump(self):
        return self.tolist()

    def load(self, x):
        self[:] = x
    
    def conj(self):
        '''conjugate'''
        res = Quaternion()
        res.real = self.real
        res.imag = -self.imag

        return res

    @property
    def real(self): return self[-1]

    @real.setter
    def real(self, value): self[-1] = value

    @property
    def imag(self):
        return np.array( self[:3] )

    @imag.setter
    def imag(self, value): self[:3] = value

    def normalize(self):
        '''normalize quaternion'''
        self /= math.sqrt( self.dot(self) )

    def flip(self):
        '''flip quaternion in the real positive halfplane, if needed'''
        res = Quaternion()
        res[:] = -self if self.real < 0 else self
        return res
    
    def __mul__(self, other):
        '''quaternion product'''
        res = Quaternion()
        res.real = self.real * other.real - self.imag.dot(other.imag)
        res.imag = self.real * other.imag + other.real * self.imag + np.cross(self.imag, other.imag)
        
        return res
         

    def __call__(self, x):
        '''rotate a vector. self should be normalized'''
        
        tmp = Quaternion()
        tmp.real = 0
        tmp.imag = x

        return (self * tmp * self.conj()).imag

    @staticmethod
    def hat(x):
        '''skew-symmetric cross-product matrix'''
        return np.array( [
            [0, -x[2], x[1]],
            [x[2], 0, -x[0]],
            [-x[1], x[0], 0]
            ])

    # 1 - 2 q_j^2 - 2 q_k^2 & 2 (q_i q_j + q_k q_r) & 2 (q_i q_k - q_j q_r) \\
    # 2 (q_i q_j - q_k q_r) & 1 - 2 q_i^2 - 2 q_k^2 & 2 (q_j q_k + q_i q_r) \\
    # 2 (q_i q_k + q_j q_r) & 2 (q_j q_k - q_i q_r) & 1 - 2 q_i^2 - 2 q_j^2 \\

    def matrix(self):
        '''quaternion to rotation matrix'''
       
        qr = self.real
        qi = self.imag[0]
        qj = self.imag[1]
        qk = self.imag[2]

        qii = qi * qi
        qjj = qj * qj
        qkk = qk * qk

        qij = qi * qj
        qkr = qk * qr
        qik = qi * qk
        qjr = qj * qr
        qjk = qj * qk
        qir = qi * qr
        
        return np.array([
            [ 1.0 - 2 * (qjj + qkk), 2 * (qij - qkr), 2 * (qik + qjr)],
            [ 2*(qij + qkr), 1.0 - 2 * (qii + qkk), 2 * (qjk - qir)],
            [ 2 * (qik - qjr), 2 * (qjk + qir), 1.0 - 2 * (qii + qjj) ]
            ])


    def angle(self):
        sign = 1 if self.real >= 0 else -1
        
        w = sign * self.real
        if w > 1: w = 1
        
        half_theta = math.acos( w )
        return 2 * half_theta
    
    @staticmethod
    def exp(x):
        '''quaternion exponential (half)'''

        theta = norm(x)

        res = Quaternion()
        
        if math.fabs(theta) < sys.float_info.epsilon:
            return res

        s = math.sin(theta / 2.0)
        c = math.cos(theta / 2.0)

        res.real = c
        res.imag = x * (s / theta)

        return res

    def log(self):
        '''(principal) logarithm'''

        q = self.flip()
        theta = q.angle()
        
        if math.fabs( theta ) < sys.float_info.epsilon:
            return np.zeros(3)

        return (theta / math.sin(theta / 2.0)) * q.imag
    


    def proj(self, n):
        '''geodesic projection (n must be unit length)'''

        dot = self.imag.dot(n)
        
        if self.real != 0:
            theta = math.atan( dot / self.real )

            # quaternion: theta
            # rotation: 2 * theta (exp will half)
            return 2 * theta
        
        elif dot != 0:
            theta = math.atan( self.real / dot )
            return 2 * (math.pi / 2.0 - theta)
        else:
            return math.pi / 2.0
    
    def euler(self, order = xrange(3) ):
        '''not good though :-/'''
        q = Quaternion()
        q[:] = self

        res = np.zeros(3)
        
        for j, i in enumerate(order):
            n = basis(i, 3)
            theta = q.proj( n )
            e = Quaternion.exp( -theta * n  )
            q[:] = e * q
            res[i] = theta
            # print q.angle()

        return  res
        
        
