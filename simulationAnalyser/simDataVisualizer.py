#!/usr/bin/env python

from os import listdir
from os.path import isfile, join, isdir
import psutil
from argparse import ArgumentParser

logFileTypes = {
    "position_and_velocity_": "posVelLog",
    "auxiliaries": "auxLog",
    "velocities_and_accelerations_": "velAccLog",
    "pin_data_and_mobilizer_reactions_": "pinMobReac",
    "rigid_body_forces_": "rigidForceLog",
    "force_set_": "forceSetLog",
    "contact_": "contactLog",
    "force_coord_CoordForce_": "coordForceLog",
    "position_only_" : "posLog",
    "velocity_only_" : "velLog"
}


parser = ArgumentParser()
parser.add_argument("-in", "--log_directory",
                  action="store", dest="inputDir", help="Directory which contains the logfile to process.")
parser.add_argument("-out", "--image_directory",
                  action="store", dest="outputDir", help="Directory for the created images.")

args = parser.parse_args()

logDir = args.inputDir
outDir = args.outputDir

if (logDir is None) or (not isdir(logDir)):
    print("\nNo valid log directory provided.\n")
    parser.print_help()
    exit()

if (outDir is None) or not isdir(outDir):
    print("\nNo valid output directory provided.\n")
    parser.print_help()
    exit()

gnuplotGeneralSettings = 'set key;' \
                         'set term png size 1200,800;'

#tmin=""
#tmax=""
#xmin=""
#xmax=""

tRange = '[]'
xRange = '[]'

curveWidth = "1"
pointSize = "0.7"
pointType = "2"

gnuplotMaxTime = 120

def executeGnuplot(filename, gnuplotCommands, title='', outfileSuffix = '', fixColon=True):
        #outfile = filename.replace('.log','.png')

        outfile = filename[:len(filename)-4] + outfileSuffix + '.png'
        if (fixColon):
            outfile = outfile.replace("::","__")

        print("Creating {0}".format(outfile))

        gnuplotTitle = ''
        if title != '':
            gnuplotTitle = 'set title "' + title + ' ('+filename+')" noenhanced;'

        gnuplotOutput = 'set out "' + join(outDir, outfile) + '";'

        try:
            #p1 = psutil.Popen([join(logDir, "gnuplot"),gnuplotString],env=env_main,stdout=outf)
            p1 = psutil.Popen(['gnuplot','-e', gnuplotGeneralSettings + gnuplotTitle + gnuplotOutput + gnuplotCommands])
            p1.wait(gnuplotMaxTime)
        except psutil.TimeoutExpired:
            print("WARNING: gnuplot has taken longer than {0} seconds trying to create {1}. "
                  "Something is probably wrong, unless the datafile is very big.".format(gnuplotMaxTime, outfile))
            p1.wait()

        print("Finished {0}\n".format(outfile))


def generateImage(type, filename):

    print("Generating image for log of type {}".format(type))

    # check if there is anything in the file except comment lines
    lgFile = open(join(logDir, filename),'r')
    lgLine = lgFile.readline().strip()
    while (len(lgLine) > 0) and (lgLine.startswith('#')):
        lgLine = lgFile.readline().strip()
    lgFile.close()
    if (len(lgLine) == 0) or (not (lgLine[0]).isdigit()):
        print("No data in {0}\n".format(filename))
        return

    dataFileString = '"' + join(logDir, filename) + '"'
    #dataRangesString = '['+tmin+':'+tmax+']['+xmin+':'+xmax+']'
    dataRangesString = tRange + xRange

    if type == "posVelLog":
        # OpenSim/Simbody dynamics data log file
        # data columns in this file:
        # time - position - velocity

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:2 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "red" title "position", ' \
                         + dataFileString + ' using 1:2 w l lw ' + curveWidth + ' lc rgb "red" notitle, ' \
                         + dataFileString + ' using 1:3 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "green" title "velocity", ' \
                         + dataFileString + ' using 1:3 w l lw ' + curveWidth + ' lc rgb "green" notitle ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Object position and velocity")

    elif type == "posLog":
        # OpenSim/Simbody dynamics data log file
        # logged data from the Q (position) vector, entry number 1
        # data columns in this file:
        # time - position

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:2 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "red" title "position", ' \
                         + dataFileString + ' using 1:2 w l lw ' + curveWidth + ' lc rgb "red" notitle, ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Object position")

    elif type == "velLog":
        # OpenSim/Simbody dynamics data log file
        # logged data from the U (velocities) vector, entry number 1
        # data columns in this file:
        # time - velocity

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:2 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "green" title "velocity", ' \
                         + dataFileString + ' using 1:2 w l lw ' + curveWidth + ' lc rgb "green" notitle ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Object velocity")

    elif type == "auxLog":
        print("auxLog-image output not implemented")

    elif type ==  "velAccLog":
        # OpenSim/Simbody dynamics data log file
        # logged data from Mobod_1
        # data columns in this file:
        # time - angular velocity - translational velocity - angular acceleration - translational acceleration

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:2 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "velocity x", ' \
                         + dataFileString + ' using 1:2 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                         + dataFileString + ' using 1:3 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-green" title "velocity y", ' \
                         + dataFileString + ' using 1:3 w l lw ' + curveWidth + ' lc rgb "dark-green" notitle, ' \
                         + dataFileString + ' using 1:4 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-blue" title "velocity z", ' \
                         + dataFileString + ' using 1:4 w l lw ' + curveWidth + ' lc rgb "dark-blue" notitle, ' \
                         + dataFileString + ' using 1:8 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "red" title "acceleration x", ' \
                         + dataFileString + ' using 1:8 w l lw ' + curveWidth + ' lc rgb "red" notitle, ' \
                         + dataFileString + ' using 1:9 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "green" title "acceleration y", ' \
                         + dataFileString + ' using 1:9 w l lw ' + curveWidth + ' lc rgb "green" notitle, ' \
                         + dataFileString + ' using 1:10 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "blue" title "acceleration z", ' \
                         + dataFileString + ' using 1:10 w l lw ' + curveWidth + ' lc rgb "blue" notitle ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Object angular velocity and angular acceleration", "_angular")

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:5 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "velocity x", ' \
                         + dataFileString + ' using 1:5 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                         + dataFileString + ' using 1:6 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-green" title "velocity y", ' \
                         + dataFileString + ' using 1:6 w l lw ' + curveWidth + ' lc rgb "dark-green" notitle, ' \
                         + dataFileString + ' using 1:7 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-blue" title "velocity z", ' \
                         + dataFileString + ' using 1:7 w l lw ' + curveWidth + ' lc rgb "dark-blue" notitle, ' \
                         + dataFileString + ' using 1:11 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "red" title "acceleration x", ' \
                         + dataFileString + ' using 1:11 w l lw ' + curveWidth + ' lc rgb "red" notitle, ' \
                         + dataFileString + ' using 1:12 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "green" title "acceleration y", ' \
                         + dataFileString + ' using 1:12 w l lw ' + curveWidth + ' lc rgb "green" notitle, ' \
                         + dataFileString + ' using 1:13 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "blue" title "acceleration z", ' \
                         + dataFileString + ' using 1:13 w l lw ' + curveWidth + ' lc rgb "blue" notitle ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Object translational velocity and translational acceleration", "_translational")

    elif type ==  "pinMobReac":
        # OpenSim/Simbody dynamics data log file
        # logged data from Mobod_0
        # data columns in this file:
        # time - pin angle - pin rate - reaction on body at mobilizer frame (moment and force) - reaction on parent at mobilizer frame (moment and force)

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:2 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "angle", ' \
                         + dataFileString + ' using 1:2 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                         + dataFileString + ' using 1:3 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-green" title "rate", ' \
                         + dataFileString + ' using 1:3 w l lw ' + curveWidth + ' lc rgb "dark-green" notitle, ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Pin joint angle and turn rate", "_pin")

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:4 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "moment (rot) x", ' \
                         + dataFileString + ' using 1:4 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                         + dataFileString + ' using 1:5 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-green" title "moment (rot) y", ' \
                         + dataFileString + ' using 1:5 w l lw ' + curveWidth + ' lc rgb "dark-green" notitle, ' \
                         + dataFileString + ' using 1:6 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-blue" title "moment (rot) z", ' \
                         + dataFileString + ' using 1:6 w l lw ' + curveWidth + ' lc rgb "dark-blue" notitle, ' \
                         + dataFileString + ' using 1:7 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "red" title "force (tran) x", ' \
                         + dataFileString + ' using 1:7 w l lw ' + curveWidth + ' lc rgb "red" notitle, ' \
                         + dataFileString + ' using 1:8 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "green" title "force (tran) y", ' \
                         + dataFileString + ' using 1:8 w l lw ' + curveWidth + ' lc rgb "green" notitle, ' \
                         + dataFileString + ' using 1:9 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "blue" title "force (tran) z", ' \
                         + dataFileString + ' using 1:9 w l lw ' + curveWidth + ' lc rgb "blue" notitle ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Mobilizer reaction on body at mobilizer frame", "_bodyReaction")

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:10 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "moment (rot) x", ' \
                         + dataFileString + ' using 1:10 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                         + dataFileString + ' using 1:11 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-green" title "moment (rot) y", ' \
                         + dataFileString + ' using 1:11 w l lw ' + curveWidth + ' lc rgb "dark-green" notitle, ' \
                         + dataFileString + ' using 1:12 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-blue" title "moment (rot) z", ' \
                         + dataFileString + ' using 1:12 w l lw ' + curveWidth + ' lc rgb "dark-blue" notitle, ' \
                         + dataFileString + ' using 1:13 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "red" title "force (tran) x", ' \
                         + dataFileString + ' using 1:13 w l lw ' + curveWidth + ' lc rgb "red" notitle, ' \
                         + dataFileString + ' using 1:14 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "green" title "force (tran) y", ' \
                         + dataFileString + ' using 1:14 w l lw ' + curveWidth + ' lc rgb "green" notitle, ' \
                         + dataFileString + ' using 1:15 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "blue" title "force (tran) z", ' \
                         + dataFileString + ' using 1:15 w l lw ' + curveWidth + ' lc rgb "blue" notitle ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Mobilizer reaction on parent at mobilizer frame", "_parentReaction")

    elif type == "rigidForceLog":
        # OpenSim/Simbody dynamics data log file
        # logged data from rigid force vector 0
        # data columns in this file:
        # time - torque - force

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:2 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "torque x", ' \
                         + dataFileString + ' using 1:2 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                         + dataFileString + ' using 1:3 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-green" title "torque y", ' \
                         + dataFileString + ' using 1:3 w l lw ' + curveWidth + ' lc rgb "dark-green" notitle, ' \
                         + dataFileString + ' using 1:4 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-blue" title "torque z", ' \
                         + dataFileString + ' using 1:4 w l lw ' + curveWidth + ' lc rgb "dark-blue" notitle ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Rigid body torque","_torque")

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:5 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "red" title "force x", ' \
                         + dataFileString + ' using 1:5 w l lw ' + curveWidth + ' lc rgb "red" notitle, ' \
                         + dataFileString + ' using 1:6 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "green" title "force y", ' \
                         + dataFileString + ' using 1:6 w l lw ' + curveWidth + ' lc rgb "green" notitle, ' \
                         + dataFileString + ' using 1:7 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "blue" title "force z", ' \
                         + dataFileString + ' using 1:7 w l lw ' + curveWidth + ' lc rgb "blue" notitle ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Rigid body force","_force")


    elif type == "forceSetLog":
        # OpenSim/Simbody dynamics data log file
        # logged data from force named foot_ground_collision
        # data columns in this file:
        # time - foot_ground_collision.foot.force.X - foot_ground_collision.foot.force.Y - foot_ground_collision.foot.force.Z - foot_ground_collision.foot.torque.X - foot_ground_collision.foot.torque.Y - foot_ground_collision.foot.torque.Z - foot_ground_collision.ground.force.X - foot_ground_collision.ground.force.Y - foot_ground_collision.ground.force.Z - foot_ground_collision.ground.torque.X - foot_ground_collision.ground.torque.Y - foot_ground_collision.ground.torque.Z
        ### or ###
        # OpenSim/Simbody dynamics data log file
        # logged data from force named ankle_pin_joint_limit
        # data columns in this file:
        # time - ankle_pin_joint_limit - PotentialEnergy
        ### possibly others, when other force types are involved

        # We don't know how many data columns there will be, so we just draw plots which contain a maximum of three
        # curves until we have drawn all the data. The log file should contain a line with force labels, which we can
        # use to label curves in the data plots

        # get the data column description line
        lgFile = open(join(logDir, filename),'r')
        lgLine = lgFile.readline().strip()
        while (len(lgLine) > 0) and not (lgLine.startswith('# time')):
            lgLine = lgFile.readline().strip()
        lgFile.close()

        if (len(lgLine) == 0):
            print("ERROR: Could not create images for {0} because the line containing the"
                  " force labels could not be found (needs to start with '# time',"
                  " followed by the labels separated with '-')".format(filename))
            return

        curveColors = ['red','green','blue']

        columnLabels = lgLine.split('-')[1:] # the first label is '# time', which is not needed
        currentDataColumn = 2

        # go over all force labels and draw an image every three labels
        gnuplotCommandString = 'plot' + dataRangesString + ' '
        curveCount = 0
        for currentLabel in columnLabels:
            gnuplotCommandString = gnuplotCommandString\
                                   + dataFileString + ' using 1:'+str(currentDataColumn)+' w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "'+curveColors[curveCount]+'" title "'+currentLabel.strip()+'" noenhance, ' \
                                   + dataFileString + ' using 1:'+str(currentDataColumn)+' w l lw ' + curveWidth + ' lc rgb "'+curveColors[curveCount]+'" notitle, ' \

            currentDataColumn = currentDataColumn + 1
            curveCount = curveCount + 1

            if (curveCount == 3):
                gnuplotCommandString = gnuplotCommandString[:len(gnuplotCommandString)-2] + ';'
                #print(gnuplotCommandString)
                executeGnuplot(filename, gnuplotCommandString, "Force values","_force_{0}_to_{1}".format(currentDataColumn-3-1,currentDataColumn-2))
                gnuplotCommandString = 'plot' + dataRangesString + ' '
                curveCount = 0

        # if the number of force labels is not divisible by three, draw the remaining curves
        if (curveCount != 0):
                #print(gnuplotCommandString)
                executeGnuplot(filename, gnuplotCommandString, "Force values","_force_{0}_to_{1}".format(currentDataColumn-curveCount-1,currentDataColumn-2))


    elif type == "contactLog":
        # OpenSim/Simbody dynamics data log file
        # logged data from a contact set with ContactSetIndex 0, corresponding to contact mesh with ContactSurfaceIndex 0
        # (note that in OpenSim a mesh-contact is always applied between the middle of a face of the intersecting mesh and the nearest point of the intersected object, which does not have to be a mesh itself)
        # data columns in this file:
        # time - contact face index - contact face distance - contact force normal

        faceIndexSet = set()

        lgFile = open(join(logDir, filename),'r')
        lgLine = lgFile.readline().strip()
        while (len(lgLine) > 0) and (not (lgLine[0]).isdigit()):
            lgLine = lgFile.readline().strip()
        while (len(lgLine) > 0):
            faceIndexSet.add(lgLine.split(' ')[1])
            lgLine = lgFile.readline().strip()

        for face in faceIndexSet:
            gnuplotCommandString = 'plot' + dataRangesString + ' '\
                             + dataFileString + ' using 1:($2)=='+face+'?($3):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "position x", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($3):1/0 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($4):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-green" title "position y", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($4):1/0 w l lw ' + curveWidth + ' lc rgb "dark-green" notitle, ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($5):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-blue" title "position z", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($5):1/0 w l lw ' + curveWidth + ' lc rgb "dark-blue" notitle ' \
                             ';'

            executeGnuplot(filename, gnuplotCommandString, "Contact point position on intersecting mesh (face #"+face+")","_face_"+face+"_posOnMesh")

            gnuplotCommandString = 'plot' + dataRangesString + ' '\
                             + dataFileString + ' using 1:($2)=='+face+'?($6):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "position x", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($6):1/0 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($7):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-green" title "position y", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($7):1/0 w l lw ' + curveWidth + ' lc rgb "dark-green" notitle, ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($8):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-blue" title "position z", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($8):1/0 w l lw ' + curveWidth + ' lc rgb "dark-blue" notitle ' \
                             ';'

            executeGnuplot(filename, gnuplotCommandString, "Contact point position on intersected mesh (face #"+face+")","_face_"+face+"_posOnOtherMesh")

            gnuplotCommandString = 'plot' + dataRangesString + ' '\
                             + dataFileString + ' using 1:($2)=='+face+'?($9):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "distance", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($9):1/0 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                             ';'

            executeGnuplot(filename, gnuplotCommandString, "Contact distance (face #"+face+")","_face_"+face+"_distance")

            gnuplotCommandString = 'plot' + dataRangesString + ' '\
                             + dataFileString + ' using 1:($2)=='+face+'?($10):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-red" title "normal x", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($10):1/0 w l lw ' + curveWidth + ' lc rgb "dark-red" notitle, ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($11):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-green" title "normal y", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($11):1/0 w l lw ' + curveWidth + ' lc rgb "dark-green" notitle, ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($12):1/0 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "dark-blue" title "normal z", ' \
                             + dataFileString + ' using 1:($2)=='+face+'?($12):1/0 w l lw ' + curveWidth + ' lc rgb "dark-blue" notitle ' \
                             ';'

            executeGnuplot(filename, gnuplotCommandString, "Contact normal (face #"+face+")","_face_"+face+"_normal")


    elif type == "coordForceLog":
        # OpenSim/Simbody dynamics data log file
        # logged data from force named sacroiliac_pin_joint_limit
        # data columns in this file:
        # time - coordinate limit force

        gnuplotCommandString = 'plot' + dataRangesString + ' '\
                         + dataFileString + ' using 1:2 w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "red" title "coordinate limit force", ' \
                         + dataFileString + ' using 1:2 w l lw ' + curveWidth + ' lc rgb "red" notitle, ' \
                         ';'

        executeGnuplot(filename, gnuplotCommandString, "Coordinate limit force")

    elif type == "dontDoAnything":
        print("Don't do anything for file {0}".format(filename))

    elif type == "Unknown":
        print("Unknown log file type: {0}\nCreating generic images.".format(filename))

        # We don't know how many data columns there will be, so we just draw plots which contain a maximum of three
        # curves until we have drawn all the data.

        # determine how many data columns there are
        lgFile = open(join(logDir, filename),'r')
        lgLine = lgFile.readline().strip()
        while (len(lgLine) > 0) and not ((lgLine[0]).isdigit()):
            lgLine = lgFile.readline().strip()
        lgFile.close()

        if (len(lgLine) == 0):
            print("ERROR: Could not create images for {0} because there is no data".format(filename))
            return

        curveColors = ['red','green','blue']

        columnsTmp = lgLine.split(' ')
        columns = []
        for tmp in columnsTmp:
            if len(tmp) > 0:
                columns.append(tmp)
        columns = columns[1:] # the first element is the time, which does not get it's own diagram

        print("Found {0} data columns (not counting the time column)".format(len(columns)))
        currentDataColumn = 2

        # draw diagrams for every for data column
        gnuplotCommandString = 'plot' + dataRangesString + ' '
        curveCount = 0
        for column in columns:
            gnuplotCommandString = gnuplotCommandString\
                                   + dataFileString + ' using 1:'+str(currentDataColumn)+' w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "'+curveColors[curveCount]+'" title "data column #'+str(currentDataColumn)+'" noenhance, ' \
                                   + dataFileString + ' using 1:'+str(currentDataColumn)+' w l lw ' + curveWidth + ' lc rgb "'+curveColors[curveCount]+'" notitle, ' \

            currentDataColumn = currentDataColumn + 1
            curveCount = curveCount + 1

            if (curveCount == 3):
                gnuplotCommandString = gnuplotCommandString[:len(gnuplotCommandString)-2] + ';'
                #print(gnuplotCommandString)
                executeGnuplot(filename, gnuplotCommandString, "Data diagram","_data_{0}_to_{1}".format(currentDataColumn-3-1,currentDataColumn-2))
                gnuplotCommandString = 'plot' + dataRangesString + ' '
                curveCount = 0

        # if the number of force labels is not divisible by three, draw the remaining curves
        if (curveCount != 0):
                #print(gnuplotCommandString)
                executeGnuplot(filename, gnuplotCommandString, "Data diagram","_data_{0}_to_{1}".format(currentDataColumn-curveCount-1,currentDataColumn-2))


    elif type == "UnknownSingle":
        print("Unknown log file type: {0}\nCreating generic images with a single image per data column.".format(filename))

        # We don't know how many data columns there will be, so we just draw plots which contain a maximum of three
        # curves until we have drawn all the data.

        # determine how many data columns there are
        lgFile = open(join(logDir, filename),'r')
        lgLine = lgFile.readline().strip()
        while (len(lgLine) > 0) and not ((lgLine[0]).isdigit()):
            lgLine = lgFile.readline().strip()
        lgFile.close()

        if (len(lgLine) == 0):
            print("ERROR: Could not create images for {0} because there is no data".format(filename))
            return

        curveColors = ['red','green','blue']

        columnsTmp = lgLine.split(' ')
        columns = []
        for tmp in columnsTmp:
            if len(tmp) > 0:
                columns.append(tmp)
        columns = columns[1:] # the first element is the time, which does not get it's own diagram

        print("Found {0} data columns (not counting the time column)".format(len(columns)))
        currentDataColumn = 2

        # draw diagrams for every for data column
        gnuplotCommandString = 'plot' + dataRangesString + ' '
        curveCount = 0
        for column in columns:
            gnuplotCommandString = 'plot' + dataRangesString + ' '\
                                   + dataFileString + ' using 1:'+str(currentDataColumn)+' w p pt ' + pointType  + ' ps ' + pointSize + ' lc rgb "'+curveColors[curveCount]+'" title "data column #'+str(currentDataColumn-1)+'" noenhance, ' \
                                   + dataFileString + ' using 1:'+str(currentDataColumn)+' w l lw ' + curveWidth + ' lc rgb "'+curveColors[curveCount]+'" notitle, ' \
                                    ';'

            executeGnuplot(filename, gnuplotCommandString, "Data diagram","_data_{0}".format(currentDataColumn-1))

            currentDataColumn = currentDataColumn + 1


#logFiles = [f for f in listdir(logDir) if ( isfile(join(logDir, f)) and (f.find(".log",len(f)-4,len(f)) != -1) )]
logFiles = [f for f in listdir(logDir) if ( isfile(join(logDir, f)) and (f.endswith(".log")) )]

# for logType in logFileTypes:
#     print(logFileTypes[logType])
#     for logFile in logFiles:
#         if (logFile.find(logType,0) == 0):
#             #print(logFile)
#             generateImage(logFileTypes[logType], logFile)

for logFile in logFiles:
    knownType = False
    for logType in logFileTypes:
        if (logFile.find(logType,0) == 0):
            #print(logFile)
            knownType = True
            generateImage(logFileTypes[logType], logFile)
    if not knownType:
        generateImage("UnknownSingle", logFile)
        asd=0

