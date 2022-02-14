import math
import inkex
import copy
import numpy
from inkex import PathElement, Style
from inkex.paths import Path,Move,Line,ZoneClose
from inkex.transforms import BoundingBox,Vector2d
from inkex.elements._groups import Group

#--------------------copied code
class pathStruct(object):
    def __init__(self):
        self.id="path0000"
        self.path=Path()
        self.enclosed=False
        self.style = None
    def __str__(self):
        return self.path
    
class pnPoint(object):
   # This class came from https://github.com/JoJocoder/PNPOLY
    def __init__(self,p):
        self.p=p
    def __str__(self):
        return self.p
    def InPolygon(self,polygon,BoundCheck=False):
        inside=False
        if BoundCheck:
            minX=polygon[0][0]
            maxX=polygon[0][0]
            minY=polygon[0][1]
            maxY=polygon[0][1]
            for p in polygon:
                minX=min(p[0],minX)
                maxX=max(p[0],maxX)
                minY=min(p[1],minY)
                maxY=max(p[1],maxY)
            if self.p[0]<minX or self.p[0]>maxX or self.p[1]<minY or self.p[1]>maxY:
                return False
        j=len(polygon)-1
        for i in range(len(polygon)):
            if ((polygon[i][1]>self.p[1])!=(polygon[j][1]>self.p[1]) and (self.p[0]<(polygon[j][0]-polygon[i][0])*(self.p[1]-polygon[i][1])/( polygon[j][1] - polygon[i][1] ) + polygon[i][0])):
                    inside =not inside
            j=i
        return inside

        

class baytop(inkex.EffectExtension):

 
    def add_arguments(self, pars):
        pars.add_argument("--usermenu")
        pars.add_argument("--unit", default="in",\
            help="Dimensional units")
        pars.add_argument("--polysides", type=int, default=6,\
            help="Number of Polygon Sides")
        pars.add_argument("--polylimit", type=int, default=0,\
            help="Number of sides on final piece,including back")
        pars.add_argument("--wantwidth", type=float, default=5.0,\
            help="Smaller width at back in dimensional units")
        pars.add_argument("--wantwidth2", type=float, default=3.0,\
            help="Larger width at back in dimensional units")
        pars.add_argument("--objht", type=float, default=2.0,\
            help="Height of piece in dimensional units")
        pars.add_argument("--dashlength", type=float, default=0.1,\
            help="Length of dashline in dimensional units (zero for solid line)")
        pars.add_argument("--tabangle", type=float, default=45.0,\
            help="Angle of tab edges in degrees")
        pars.add_argument("--linesonwrapper", type=inkex.Boolean, dest="linesonwrapper",\
            help="Put dashlines on wrappers")
        pars.add_argument("--tabheight", type=float, default=0.4,\
            help="Height of tab in dimensional units")
            
            

    #draw SVG line segment(s) between the given (raw) points
    def drawline(self, dstr, name, parent, sstr=None):
        line_style   = {'stroke':'#000000','stroke-width':'0.25','fill':'#eeeeee'}
        if sstr == None:
            stylestr = str(Style(line_style))
        else:
            stylestr = sstr
        el = parent.add(PathElement())
        el.path = dstr
        el.style = stylestr
        el.label = name
 
    # Thanks to Gabriel Eng for his python implementation of https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    def findIntersection(self, x1,y1,x2,y2,x3,y3,x4,y4):
        px= ( (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) ) 
        py= ( (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) )
        return px, py

    def insidePath(self, path, p):
        point = pnPoint((p.x, p.y))
        pverts = []
        for pnum in path:
            if pnum.letter == 'Z':
                pverts.append((path[0].x, path[0].y))
            else:
                pverts.append((pnum.x, pnum.y))
        isInside = point.InPolygon(pverts, True)
        return isInside # True if point p is inside path

    def makescore(self, pt1, pt2, dashlength1):
        # Draws a dashed line of dashlength between two points
        # Dash = dashlength space followed by dashlength mark
        # if dashlength is zero, we want a solid line
        # Returns dashed line as a Path object
        apt1 = Line(0.0,0.0)
        apt2 = Line(0.0,0.0)
        ddash = Path()
        #sue change to adjust for small spans
        if self.lineLen(Vector2d(pt1.x,pt1.y),Vector2d(pt2.x,pt2.y)) < (3*dashlength1):
            dashlength = dashlength1/2.1
        else:
            dashlength = dashlength1
        # end sue change
        if math.isclose(dashlength, 0.0):
            #inkex.utils.debug("Draw solid dashline")
            ddash.append(Move(pt1.x,pt1.y))
            ddash.append(Line(pt2.x,pt2.y))
        else:
            if math.isclose(pt1.y, pt2.y):
                #inkex.utils.debug("Draw horizontal dashline")
                if pt1.x < pt2.x:
                    xcushion = pt2.x - dashlength
                    xpt = pt1.x
                    ypt = pt1.y
                else:
                    xcushion = pt1.x - dashlength
                    xpt = pt2.x
                    ypt = pt2.y
                done = False
                while not(done):
                    if (xpt + dashlength*2) <= xcushion:
                        xpt = xpt + dashlength
                        ddash.append(Move(xpt,ypt))
                        xpt = xpt + dashlength
                        ddash.append(Line(xpt,ypt))
                    else:
                        done = True
            elif math.isclose(pt1.x, pt2.x):
                #inkex.utils.debug("Draw vertical dashline")
                if pt1.y < pt2.y:
                    ycushion = pt2.y - dashlength
                    xpt = pt1.x
                    ypt = pt1.y
                else:
                    ycushion = pt1.y - dashlength
                    xpt = pt2.x
                    ypt = pt2.y
                done = False
                while not(done):
                    if(ypt + dashlength*2) <= ycushion:
                        ypt = ypt + dashlength         
                        ddash.append(Move(xpt,ypt))
                        ypt = ypt + dashlength
                        ddash.append(Line(xpt,ypt))
                    else:
                        done = True
            else:
                #inkex.utils.debug("Draw sloping dashline")
                if pt1.y > pt2.y:
                    apt1.x = pt1.x
                    apt1.y = pt1.y
                    apt2.x = pt2.x
                    apt2.y = pt2.y
                else:
                    apt1.x = pt2.x
                    apt1.y = pt2.y
                    apt2.x = pt1.x
                    apt2.y = pt1.y
                m = (apt1.y-apt2.y)/(apt1.x-apt2.x)
                theta = math.atan(m)
                msign = (m>0) - (m<0)
                ycushion = apt2.y + dashlength*math.sin(theta)
                xcushion = apt2.x + msign*dashlength*math.cos(theta)
                xpt = apt1.x
                ypt = apt1.y
                done = False
                while not(done):
                    nypt = ypt - dashlength*2*math.sin(theta)
                    nxpt = xpt - msign*dashlength*2*math.cos(theta)
                    if (nypt >= ycushion) and (((m<0) and (nxpt <= xcushion)) or ((m>0) and (nxpt >= xcushion))):
                        # move to end of space / beginning of mark
                        xpt = xpt - msign*dashlength*math.cos(theta)
                        ypt = ypt - msign*dashlength*math.sin(theta)
                        ddash.append(Move(xpt,ypt))
                        # draw the mark
                        xpt = xpt - msign*dashlength*math.cos(theta)
                        ypt = ypt - msign*dashlength*math.sin(theta)
                        ddash.append(Line(xpt,ypt))
                    else:
                        done = True
        return ddash

    def detectIntersect(self, x1, y1, x2, y2, x3, y3, x4, y4):
        td = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
        if td == 0:
            # These line segments are parallel
            return False
        t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/td
        if (0.0 <= t) and (t <= 1.0):
            return True
        else:
            return False

    def orientTab(self,pt1,pt2,height,angle,theta,orient):
        tpt1 = Line(0.0,0.0)
        tpt2 = Line(0.0,0.0)
        tpt1.x = pt1.x + orient[0]*height + orient[1]*height/math.tan(math.radians(angle))
        tpt2.x = pt2.x + orient[2]*height + orient[3]*height/math.tan(math.radians(angle))
        tpt1.y = pt1.y + orient[4]*height + orient[5]*height/math.tan(math.radians(angle))
        tpt2.y = pt2.y + orient[6]*height + orient[7]*height/math.tan(math.radians(angle))
        if not math.isclose(theta, 0.0):
            t11 = Path([Move(pt1.x,pt1.y),Line(tpt1.x, tpt1.y)])
            t12 = Path([Move(pt1.x,pt1.y),Line(tpt2.x, tpt2.y)])
            thetal1 = t11.rotate(theta, [pt1.x,pt1.y])
            thetal2 = t12.rotate(theta, [pt2.x,pt2.y])
            tpt1.x = thetal1[1].x
            tpt1.y = thetal1[1].y
            tpt2.x = thetal2[1].x
            tpt2.y = thetal2[1].y
        return tpt1,tpt2

    def makeTab(self, tpath, pt1, pt2, tabht, taba):
        # tpath - the pathstructure containing pt1 and pt2
        # pt1, pt2 - the two points where the tab will be inserted
        # tabht - the height of the tab
        # taba - the angle of the tab sides
        # returns the two tab points (Line objects) in order of closest to pt1
        tpt1 = Line(0.0,0.0)
        tpt2 = Line(0.0,0.0)
        currTabHt = tabht
        currTabAngle = taba
        testAngle = 1.0
        testHt = currTabHt * 0.001
        adjustTab = 0
        tabDone = False
        while not tabDone:
            # Let's find out the orientation of the tab
            if math.isclose(pt1.x, pt2.x):
                # It's vertical. Let's try the right side
                if pt1.y < pt2.y:
                    pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,0.0,[1,0,1,0,0,1,0,-1])
                    if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                       (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[-1,0,-1,0,0,1,0,-1]) # Guessed wrong
                    else:
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[1,0,1,0,0,1,0,-1]) # Guessed right
                else: # pt2.y < pt1.y
                    pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,0.0,[1,0,1,0,0,-1,0,1])
                    if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                       (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[-1,0,-1,0,0,-1,0,1]) # Guessed wrong
                    else:
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[1,0,1,0,0,-1,0,1]) # Guessed right
            elif math.isclose(pt1.y, pt2.y):
                # It's horizontal. Let's try the top
                if pt1.x < pt2.x:
                    pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,0.0,[0,1,0,-1,-1,0,-1,0])
                    if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                       (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[0,1,0,-1,1,0,1,0]) # Guessed wrong
                    else:
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[0,1,0,-1,-1,0,-1,0]) # Guessed right
                else: # pt2.x < pt1.x
                    pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,0.0,[0,-1,0,1,-1,0,-1,0])
                    if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                       (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[0,-1,0,1,1,0,1,0]) # Guessed wrong
                    else:
                        tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,0.0,[0,-1,0,1,-1,0,-1,0]) # Guessed right

            else: # the orientation is neither horizontal nor vertical
                # Let's get the slope of the line between the points
                # Because Inkscape's origin is in the upper-left corner,
                # a positive slope (/) will yield a negative value
                slope = (pt2.y - pt1.y)/(pt2.x - pt1.x)
                # Let's get the angle to the horizontal
                theta = math.degrees(math.atan(slope))
                # Let's construct a horizontal tab
                seglength = math.sqrt((pt1.x-pt2.x)**2 +(pt1.y-pt2.y)**2)
                if slope < 0.0:
                    if pt1.x < pt2.x:
                        pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,theta,[0,1,0,-1,-1,0,-1,0])
                        if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                           (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,1,0,-1,1,0,1,0]) # Guessed wrong
                        else:
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,1,0,-1,-1,0,-1,0]) # Guessed right
                    else: # pt1.x > pt2.x
                        pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,theta,[0,-1,0,1,-1,0,-1,0])
                        if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                           (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,-1,0,1,1,0,1,0]) # Guessed wrong
                        else:
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,-1,0,1,-1,0,-1,0]) # Guessed right
                else: # slope > 0.0
                    if pt1.x < pt2.x:
                        pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,theta,[0,1,0,-1,-1,0,-1,0])
                        if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                           (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,1,0,-1,1,0,1,0]) # Guessed wrong
                        else:
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,1,0,-1,-1,0,-1,0]) # Guessed right
                    else: # pt1.x > pt2.x
                        pnpt1,pnpt2 = self.orientTab(pt1,pt2,testHt,testAngle,theta,[0,-1,0,+1,-1,0,-1,0])
                        if ((not tpath.enclosed) and (self.insidePath(tpath.path, pnpt1) or self.insidePath(tpath.path, pnpt2))) or \
                           (tpath.enclosed and ((not self.insidePath(tpath.path, pnpt1)) and (not self.insidePath(tpath.path, pnpt2)))):
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,-1,0,1,1,0,1,0]) # Guessed wrong
                        else:
                            tpt1,tpt2 = self.orientTab(pt1,pt2,currTabHt,currTabAngle,theta,[0,-1,0,1,-1,0,-1,0]) # Guessed right
            # Check to see if any tabs intersect each other
            if self.detectIntersect(pt1.x, pt1.y, tpt1.x, tpt1.y, pt2.x, pt2.y, tpt2.x, tpt2.y):
                # Found an intersection.
                if adjustTab == 0:
                    # Try increasing the tab angle in one-degree increments
                    currTabAngle = currTabAngle + 1.0
                    if currTabAngle > 88.0: # We're not increasing the tab angle above 89 degrees
                        adjustTab = 1
                        currTabAngle = taba
                if adjustTab == 1:
                    # So, try reducing the tab height in 20% increments instead
                    currTabHt = currTabHt - tabht*0.2 # Could this lead to a zero tab_height?
                    if currTabHt <= 0.0:
                        # Give up
                        currTabHt = tabht
                        adjustTab = 2
                if adjustTab == 2:
                    tabDone = True # Just show the failure
            else:
                tabDone = True
            
        return tpt1,tpt2
        
    


#-------------------end copied code

 
        
    def geo_c_alpha_a(self,c,alpha):
        a= c * math.sin(math.radians(alpha))
        return(a)   
        

        
    def geo_c_b_a (self,c,b):
        a=math.sqrt(c**2-b**2)
        return a
       
    def lineLen(self,p,q):
        dx = abs(p.x - q.x)
        dy = abs(p.y - q.y)
        llen = math.sqrt((dx**2)+(dy**2))
        return llen
        
    def stretch(self,sg,ht):
        return(math.sqrt((sg**2)+(ht**2)))
        
    def thetaradians(self,p1,p2):
        if math.isclose(p1.x, p2.x, abs_tol=1e-8): #avoid division by zero
            theta = 0.0
        else:
            theta = math.atan((p1.y - p2.y) / (p1.x - p2.x))
        return theta
        
    def thetadegrees(self,p1,p2):
        if math.isclose(p1.x, p2.x, abs_tol=1e-8): #avoid division by zero
            theta = 0.0
        else:
            theta = math.degrees(math.atan((p1.y - p2.y) / (p1.x - p2.x)))
        return theta
            
    def makepoly(self, toplength, numpoly,polylimit):
      plocs = []
      r = toplength/(2*math.sin(math.pi/numpoly))
      pstr = Path()
      for ppoint in range(0,polylimit):
         xn = r*math.cos(2*math.pi*ppoint/numpoly)
         yn = r*math.sin(2*math.pi*ppoint/numpoly)
         if ppoint == 0:
            startnode = Vector2d(xn,yn)
            pstr.append(Move(xn,yn))
         else:
            pstr.append(Line(xn,yn))
      endnode = Vector2d(xn,yn)
      pstr.append(ZoneClose())
      return pstr,startnode,endnode
      
    def anglepiece(self,a,b,c,d):  #bottom right top left
        j=((a**2) -  (2*a * c) - ( b**2 )+ (c**2) + (d**2 ) ) / (2*d*(a-c))
        print("j is "+str(j) +" ")
        alpha = math.acos(j)
        alphad = math.degrees(alpha)
        print ("alphad = "+str(alphad))
        delta = -alphad+180 
        e=math.sqrt( (c**2)-(2*c*d* math.cos(math.radians(delta))) + (d**2))
        
        m = (0.5*(a**2)+0.5*(b**2)-0.5*(e**2))/(a*b)
        print("m is "+str(m) +" ")
        beta = math.acos(m)
        betad = math.degrees(beta)
        gamma = -betad+180
        height = b * math.sin(beta)
        return(alphad,betad,gamma,delta,height)

        
    def effect(self):
        layer = self.svg.get_current_layer()
        scale = self.svg.unittouu('1'+self.options.unit)
        polysides = int(self.options.polysides)
        wantwidth = float(self.options.wantwidth) * scale
        wantwidth2 = float(self.options.wantwidth2) * scale
        polylimit = int(self.options.polylimit)
        objht = float(self.options.objht) * scale
        tab_angle = float(self.options.tabangle)
        tab_height = float(self.options.tabheight) * scale
        dashlength = float(self.options.dashlength) * scale
        lines_on_wrapper = self.options.linesonwrapper
        if polylimit<2:
            polylimit = polysides
        LLen = []
        polybigr = 7*scale  #nominal sizing -- will scale to proper size later later
        polysmallr = 5*scale
        
        rotval = 0
        layer = self.svg.get_current_layer()
        traplist = []  #this is going to hold instances of trapstructs
        trappath =Path()
        sidepath = pathStruct()
        bp = pathStruct()
        tbpath = Path()
        dscore = Path()
        dscore1 = Path()
        bpscore = Path()
        bp2=Path()
        polysmall,startnode,endnode = self.makepoly(polysmallr,polysides,polylimit)
        polybig, startnodeb,endnodeb =   self.makepoly(polysmallr,polysides,polylimit)
        #use the start and end nodes to determine if we need to rotate the piece
        
        rotval = (self.thetadegrees(startnode,endnode)) 
        
        #rotate if necessary
        polybig = polybig.rotate(-rotval)
        polysmall = polysmall.rotate(-rotval)
        
        #now need to resize and move into align top and center
        #resize as needed.
           
        ts = (polysmall.bounding_box())
        tb = polybig.bounding_box()
        #resize before translating
        

        reduceval =  wantwidth/(tb.right-tb.left)
        reduceval2 =  wantwidth2/(ts.right-ts.left)
        
        polybig = polybig.scale(reduceval,reduceval)
        polysmall = polysmall.scale(reduceval2,reduceval2)   
        ts = polysmall.bounding_box()
        tb = polybig.bounding_box()        
        halftslen = (ts.right - ts.left)/2
        polysmall.translate(-(halftslen+ts.left),-ts.top,inplace = True)
        halftblen = (tb.right - tb.left)/2
        polybig.translate(-(halftblen+tb.left),-tb.top,inplace = True)
        # Now resize to 3" or 
        #minx maxx miny maxy
        self.drawline(str(polybig),"polybig",layer)
        self.drawline(str(polysmall),"polysmall",layer)
        
        
        
        #construct backpiece
        
        wantdiff = (wantwidth2 - wantwidth)/2
        bp.path.append(Move(wantdiff,0))
        bp.path.append(Line(wantwidth+wantdiff,0))
        bp.path.append(Line(wantwidth2,objht))
        bp.path.append(Line(0,objht))
        bp.path.append(ZoneClose())
        #tabpt1, tabpt2 = self.makeTab(sidepath, cpt1, cpt2, tab_height, tab_angle)
        bp2.append(bp.path[0])
        tabpt1, tabpt2 = self.makeTab(bp, bp.path[0], bp.path[1], tab_height, tab_angle)
        bp2.append(tabpt1)
        bp2.append(tabpt2)
        bp2.append(bp.path[1])
        
        bp2.append(bp.path[2])
        tabpt1, tabpt2 = self.makeTab(bp, bp.path[2], bp.path[3], tab_height, tab_angle)
        bp2.append(tabpt1)
        bp2.append(tabpt2)
        bp2.append(bp.path[3])
        bp2.append(ZoneClose())
        
        bpscore = bpscore + self.makescore(bp.path[0], bp.path[1],dashlength)
        bpscore = bpscore + self.makescore(bp.path[2], bp.path[3],dashlength)
        
        
        
        if math.isclose(dashlength, 0.0):
            # lump together all the score lines
            groupbp = Group()
            groupbp.label = 'groupbp'
            self.drawline(str(bp2),'backpiece',groupbp) # Output the model
            if bpscore != '':
                self.drawline(str(bpscore),'bpscore',groupbp) # Output the scorelines separately
            layer.append(groupbp)
        
        else:
            bp2 = bp2+bpscore
            self.drawline(str(bp2),"backpanel",layer)
        
        
        
        smpolyside = self.lineLen((Vector2d(polysmall[0].x,polysmall[0].y)),Vector2d(polysmall[1].x,polysmall[1].y))     
        bigpolyside = self.lineLen((Vector2d(polybig[0].x,polybig[0].y)),Vector2d(polybig[1].x,polybig[1].y))  
        
        for j in range(polylimit):
            a=Vector2d(polysmall[j].x,polysmall[j].y)
            b=Vector2d(polybig[j].x,polybig[j].y)
            LLen.append(self.stretch(self.lineLen(a,b),objht))
        prevrot = 0
        for j in range(polylimit-1):  #
            
            alpha,beta,gamma,delta, ht = self.anglepiece(bigpolyside,LLen[j],smpolyside,LLen[j+1])   #bottom right top left
            if j == 0:
                prevalpha = alpha
            #inkex.utils.debug("alpha is {}".format(str(alpha)))
            #inkex.utils.debug("bottom={} right={}  top = {}  left={}".format(str(bigpolyside),str(LLen[j]),str(smpolyside),str(LLen[j+1])))
           
            troffset = self.geo_c_b_a(LLen[j+1],ht)
            if alpha>90:
                troffset = -troffset
            #make path of two line segments
            top1 = (-bigpolyside+troffset)+smpolyside
            top2 = (-bigpolyside+troffset)
            # Move 0,0 Line -bigpolyside,0 Move (top1,ht) Line(top2,ht)
            trappath.append(Line(0,0))  # was Move
            trappath.append(Line (-bigpolyside,0))
            trappath.append(Move (top1,-ht))
            trappath.append(Line(top2,-ht))
            traplist.append(copy.deepcopy(trappath))
            
           
            
            if j>0:
            
                #translate 
                x0 = traplist[j][0].x      #current lower right x
                y0 = traplist[j][0].y      #current lower right y
                xp3 = traplist[j-1][3].x   #previous upper left x
                yp3 = traplist[j-1][3].y   #previous upper left y
                xp1 = traplist[j-1][1].x   #previous lower left x
                yp1 = traplist[j-1][1].y   #previous lower left y
                
                #TRANSLATE
                traplist[j].translate(xp1,yp1,inplace=True)
                x0 = traplist[j][0].x
                y0 = traplist[j][0].y
                #rotate
                rotamt = (90-prevalpha) + (90-beta)+ prevrot
                
                #ROTATE
                traplist[j].rotate(rotamt,Vector2d(x0,y0),inplace=True)

                prevalpha = alpha
                prevrot = rotamt
       
            #self.drawline(str(traplist[j]),"traplist"+str(j),layer)
            #create path
            
            #inkex.utils.debug("traplist {} ={}".format(str(j),str(traplist[j])))
            
            
            trappath.clear()

        #make the sidepath -- lets make it a pathStruct
        
        for j in range(polylimit-1):

            if j==0:
                sidepath.path.append(Move (traplist[j][2].x, traplist[j][2].y))
            sidepath.path.append(Line (traplist[j][3].x, traplist[j][3].y))
        pointone = True  
        for j in reversed(range(polylimit-1)):
            if pointone:
                sidepath.path.append(Line(traplist[j][1].x,traplist[j][1].y))
                pointone = False   
            sidepath.path.append(Line (traplist[j][0].x, traplist[j][0].y))
        sidepath.path.append(ZoneClose())
        tblen = len(sidepath.path)  #e.g. 11
        for j in range(1,(tblen//2) - 1):
            cpt1 = sidepath.path[j]
            cpt2 = sidepath.path[(tblen-2)-j]
            dscore1 = dscore1 + self.makescore(cpt1, cpt2,dashlength)
       
        #inkex.utils.debug("length sidepath is "+str(len(sidepath)))
        #need a pathstruct for tabs
        
        tbpath.append(copy.deepcopy(sidepath.path[0]))
        for j in range(1,tblen)  :
            
            #inkex.utils.debug ("tbpath[{}] is {}".format(j,str(tbpath[j])))
             
            cpt1 = sidepath.path[j-1]  #Move or Line
            cpt2 = sidepath.path[j]    #Line or Z
            
            if str(cpt2) == "Z":
                cpt2 = Line(sidepath.path[0].x,sidepath.path[0].y) #use first point
            tabpt1, tabpt2 = self.makeTab(sidepath, cpt1, cpt2, tab_height, tab_angle)
           
            tbpath.append(tabpt1)
            tbpath.append(tabpt2)
            tbpath.append(cpt2)
            tbpath.append(copy.deepcopy(sidepath.path[j]))
            
            dscore = dscore + self.makescore(cpt1, cpt2,dashlength)
        
        #and scores between j and tabbeden-(j+1)
        dscore = dscore + dscore1
        
        if math.isclose(dashlength, 0.0):
            # lump together all the score lines
            group = Group()
            group.label = 'group'+'ms'
            self.drawline(str(tbpath),'model',group) # Output the model
            if dscore != '':
                self.drawline(str(dscore),'score',group) # Output the scorelines separately
            layer.append(group)
            
            # lump together all the score lines
            group2 = Group()
            group2.label = 'group2'+'ms2'
            self.drawline(str(sidepath.path),'model2',group2) # Output the model
            if dscore1 != '':
                self.drawline(str(dscore1),'score2',group2) # Output the scorelines separately
            layer.append(group2)
             
        else:
            tbpath = tbpath + dscore
            self.drawline(str(tbpath),'ms',layer)
            sidepath.path = sidepath.path + dscore1
            self.drawline(str(sidepath.path),'ms2',layer)

        
        #dscore = dscore + self.makescore(apath.path[ptn], apath.path[ptn+1],dashlength)
        
        
        kht = sidepath.path.bounding_box()
        tx = kht.width
        ty = kht.height
        sidepath.path.translate(tx,ty,inplace=True)
              
        #we have all the basic pieces 
        #need to make copy of sidepath with tabs and score lines 
        #need to make copy of back with tabs

    
        
if __name__ == '__main__':
    baytop().run()
