import numpy as np


class TrajObj():
    def __init__( self, point1 , point2 , vel1 , vel2 , dt = 0.1):
        self.point1 = np.array(point1)
        self.point2 = np.array(point2)
        self.vel1 = np.array(vel1)
        self.vel2 = np.array(vel2)
        self.total = 1000
        self.current = 0
        self.coef = np.zeros(4)
        self.flag = True
        self.cal_coef()
        self.dt = dt

    def cal_coef( self):
        t = self.total
        mat = np.array( [
            [ 0  , 0 ,  0 ,  1],
            [ t**3 , t**2 , t , 1],
            [0 , 0, 1 , 0],
            [3*t**2 , t*2 , 1 , 0]
        ])
        self.coef =  np.dot ( np.linalg.inv(mat), np.array([self.point1, self.point2 , self.vel1 ,self.vel2] )  )
        self.a , self.b , self.c , self.d = self.coef

    def getNextpoint( self ) :
        t = self.current
        pos = self.a * t**3 + self.b * t**2 + self.c * t + self.d 
        vel = 3*self.a * t**2 + 2*self.b * t + self.c 
        axcl = 6 * self.a + 2*self.b
        if self.flag :
            self.current += self.dt
            self.flag = not ( self.current >= 1001)
        else :
            self.current -= self.dt
            self.flag = ( self.current <= self.dt )
  
        # self.current = self.current % self.total
        return pos, vel , axcl

    
if __name__ == "__main__":

    import matplotlib.pyplot as plt

    pos_array = []
    vel_array = []
    t = TrajObj( [3,2,1.5] , [1,2,3] , [0,0,0] , [0,0,0])
    # print( t.getNextpoint(  ) )
    print( t.coef)
    for _ in range( 1000):
        pos, vel = t.getNextpoint()
        pos_array.append( pos )
        vel_array.append( vel)

    plt.plot(  pos_array)
    plt.show()
    plt.plot ( vel_array)
    plt.show()