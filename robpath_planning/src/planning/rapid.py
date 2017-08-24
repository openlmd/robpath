import ftplib
import os
import rospkg

class Rapid():
    def __init__(self):
        self.host = '192.168.30.4'
        self.user = 'anonymous'
        self.password = 'anonymous@'

        self.set_process(8, 1200)
        self.set_powder(3, 20, 20)

        self.travel_speed = 'v50'
        self.speed_t = 50
        self.travel_zone = 'z0'

        #self.tool = [[351.106, -36.6277, 86.9243], [0.70711, 0.0, -0.70711, 0.0]] # Tool pose
        self.tool = [[351.1,-36.6,86.9],[-0.5000, -0.0000, 0.8660, -0.0000]] # Tool pose 60
        #self.workobject = [[1655, -87, 932], [1, 0, 0, 0]] # Work Object pose
        self.workobject = [[1255, -87, 1032], [1, 0, 0, 0]] # Work Object pose 60

    def set_process(self, speed, power, travel=50):
        self.speed = speed
        self.power = power
        self.speed_t = travel


    def set_powder(self, carrier, stirrer, turntable):
        self.carrier = carrier
        self.stirrer = stirrer
        self.turntable = turntable

    def path2rapid(self, path):
        RAPID_TEMPLATE  = 'MODULE Etna\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    PERS tooldata toolEtna:=%(tool)s;\n'
        RAPID_TEMPLATE += '    PERS wobjdata wobjEtna:=%(wobj)s;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    VAR triggdata laserON;\n'
        RAPID_TEMPLATE += '    VAR triggdata laserOFF;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    CONST speeddata vl:=[%(speed)i,500,5000,1000];\n' #CONST speeddata v15 := [15,500,5000,1000];
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    %(targets)s\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += 'PROC cladding()\n'
        RAPID_TEMPLATE += '    %(moves)s\n'
        RAPID_TEMPLATE += 'ENDPROC\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += 'PROC mainEtna()\n'
        RAPID_TEMPLATE += '    Set Do_RF_MainOn;\n'
        RAPID_TEMPLATE += '    Set Do_RF_StandByOn;\n'
        RAPID_TEMPLATE += '    WaitDI DI_RF_LaserBeamReady,1;\n'
        RAPID_TEMPLATE += '    WaitDI DI_RF_GeneralFault,0;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    SetGO GO_Program_Rf, 0;\n' # set the program for control of laser power - prog 5
        RAPID_TEMPLATE += '    WaitTime 1;\n'
        RAPID_TEMPLATE += '    !SetGO GoLDL_Pwr3, %(power)i;\n' # set the laser power - 2200 W
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    TriggIO laserON, 0\DOp:=Do_RF_ExterGate, 1;\n'
        RAPID_TEMPLATE += '    TriggIO laserOFF, 0\DOp:=Do_RF_ExterGate, 0;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    Set DoWeldGas;\n'
        RAPID_TEMPLATE += '    !MedicoatL2 "OFF", 5, 20, 7.5;\n' # gas de arrastre, stirrer, turntable
        RAPID_TEMPLATE += '    MedicoatL1 "OFF", %(carrier)i, %(stirrer)i, %(turntable)i;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    ConfL \Off;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    MoveL [[0.0,0.0,100.0],[1.0,0.0,0.0,0.0],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v80,z0,toolEtna\WObj:=wobjEtna;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    cladding;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    MoveL [[0.0,0.0,100.0],[1.0,0.0,0.0,0.0],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]],v80,z0,toolEtna\WObj:=wobjEtna;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    Reset doMdtPL2On;\n'
        RAPID_TEMPLATE += '    Reset doMdtPL1On;\n'
        RAPID_TEMPLATE += '    Reset DoWeldGas;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += '    Reset Do_RF_StandByOn;\n'
        RAPID_TEMPLATE += '    !Reset Do_RF_MainOn;\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += 'ENDPROC\n'
        RAPID_TEMPLATE += '\n'
        RAPID_TEMPLATE += 'ENDMODULE\n'
        # Target points definition
        targets = ''
        for k in range(len(path)):
            p, q, b = path[k]
            targets = '\n'.join([targets, '    CONST robtarget T%i:=[[%f,%f,%f],[%f,%f,%f,%f],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];' %(k, p[0], p[1], p[2], q[3], q[0], q[1], q[2])])
        # Movement definition
        moves = '!Reset doLDLStartST;\n'
        for k in range(len(path)):
            p, q, process = path[k]
            if process:
                #moves = '\n'.join([moves, '    Set doLDLStartST;'])
                #moves = '\n'.join([moves, '    MoveL T%i,vl,z0,toolEtna\WObj:=wobjEtna;' %(k)])
                if k < len(path)-1 and path[k+1][2]:
                    # If the track continues in the next point. Don't OFF the laser.
                    moves = '\n'.join([moves, '    TriggL T%i,vl,laserON,z0,toolEtna\WObj:=wobjEtna;' % (k)])
                else:
                    moves = '\n'.join([moves, '    TriggL T%i,vl,laserOFF,z0,toolEtna\WObj:=wobjEtna;' % (k)])
            else:
                #moves = '\n'.join([moves, '    Reset doLDLStartST;'])
                #moves = '\n'.join([moves, '    MoveL T%i,%s,%s,toolEtna\WObj:=wobjEtna;' %(k, self.travel_speed, self.travel_zone)])
                moves = '\n'.join([moves, '    TriggL T%i,%s,laserON,%s,toolEtna\WObj:=wobjEtna;' %(k, self.travel_speed, self.travel_zone)])
        moves = '\n'.join([moves, '\n!Reset doLDLStartST;'])

        tool = '[TRUE,[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]],[20,[70,30,123.5],[0,0,1,0],1,0,1]]' %(self.tool[0][0], self.tool[0][1], self.tool[0][2], self.tool[1][0], self.tool[1][1], self.tool[1][2], self.tool[1][3])
        wobj = '[FALSE,TRUE,"",[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]],[[0,0,0],[1,0,0,0]]]' %(self.workobject[0][0], self.workobject[0][1], self.workobject[0][2], self.workobject[1][0], self.workobject[1][1], self.workobject[1][2], self.workobject[1][3])

        return RAPID_TEMPLATE %{'tool': tool,
                                'wobj': wobj,
                                'speed': self.speed,
                                'targets': targets,
                                'moves': moves,
                                'carrier': self.carrier,
                                'stirrer': self.stirrer,
                                'turntable': self.turntable,
                                'power': self.power}

    def rapid_laser_conf(self, laser_type):
        LASER_TEMPLATE = ''
        if laser_type == 'rofin_rf':
            LASER_TEMPLATE += 'Set Do_RF_MainOn;\n'
            LASER_TEMPLATE += '    Set Do_RF_StandByOn;\n'
            LASER_TEMPLATE += '    WaitDI DI_RF_LaserBeamReady,1;\n'
            LASER_TEMPLATE += '    WaitDI DI_RF_GeneralFault,0;\n'
            LASER_TEMPLATE += '\n'
            LASER_TEMPLATE += '    SetGO GO_Program_Rf, 0;\n' # set the program for control of laser power - prog 5
            LASER_TEMPLATE += '    WaitTime 1;\n'
            LASER_TEMPLATE += '    !SetGO GoLDL_Pwr3, %(power)i;\n' # set the laser power - 2200 W
        elif laser_type == 'trudisk':
            LASER_TEMPLATE += 'Set TdoExtActiv;\n'
            LASER_TEMPLATE += '    WaitDi TdiExtActiv, 1;\n'
            LASER_TEMPLATE += '    Set TdoLaserOn;\n'
            LASER_TEMPLATE += '    WaitDi TdiLaserOn, 1;\n'
            LASER_TEMPLATE += '    Set TdoStandBy;\n'
            LASER_TEMPLATE += '    Set TdoActLaser;\n'
            LASER_TEMPLATE += '    SetGO TGOPROGRAM_No, 28;\n'
            LASER_TEMPLATE += '    WaitDi TdiLaserAsig, 1;\n'
            LASER_TEMPLATE += '    WaitTime 2;\n'
            LASER_TEMPLATE += '    WaitDi TdiLaserReady, 1;\n'
        return LASER_TEMPLATE %{'power': self.power}

    def rapid_laser_stop(self, laser_type):
        LASER_TEMPLATE = ''
        if laser_type == 'rofin_rf':
            LASER_TEMPLATE += 'Reset Do_RF_StandByOn;'
        elif laser_type == 'trudisk':
            LASER_TEMPLATE += 'Reset TdoActLaser;\n'
            LASER_TEMPLATE += '    Reset TdoStandBy;\n'
            LASER_TEMPLATE += '    Reset TdoLaserOn;\n'
            LASER_TEMPLATE += '    Reset TdoExtActiv;'
        return LASER_TEMPLATE

    def rapid_feeder_conf(self, feeder_type):
        FEEDER_TEMPLATE = 'Set DoWeldGas;\n'
        if feeder_type == 'medicoat':
            FEEDER_TEMPLATE += '    !MedicoatL2 "OFF", 5, 20, 7.5;\n' # gas de arrastre, stirrer, turntable
            FEEDER_TEMPLATE += '    MedicoatL1 "OFF", %(carrier)i, %(stirrer)i, %(turntable)i;'
        elif feeder_type == 'gtv':
            FEEDER_TEMPLATE += '    SetAO AoGTV_ExternDisk, 35;\n'
            FEEDER_TEMPLATE += '    SetAO AoGTV_ExternMassFlow, 26.6;\n'
            FEEDER_TEMPLATE += '    Set doGTV_StartExtern;\n'
            FEEDER_TEMPLATE += '    WaitTime 15;'
        return FEEDER_TEMPLATE %{'carrier': self.carrier,
                                'stirrer': self.stirrer,
                                'turntable': self.turntable}

    def rapid_feeder_stop(self, feeder_type):
        FEEDER_TEMPLATE = ''
        if feeder_type == 'medicoat':
            FEEDER_TEMPLATE += 'Reset doMdtPL2On;\n'
            FEEDER_TEMPLATE += '    Reset doMdtPL1On;\n'
        elif feeder_type == 'gtv':
            FEEDER_TEMPLATE += 'Reset doGTV_StartExtern;\n'
        FEEDER_TEMPLATE += '    Reset DoWeldGas;'
        return FEEDER_TEMPLATE

    def load_template(self):
        rospack = rospkg.RosPack()
        templante_path = rospack.get_path('robpath_planning')
        templante_name = templante_path + '/templates/rapid.txt'
        with open(templante_name) as file:
            lines = file.readlines()
            RAPID_TEMPLATE = ''
            for line in lines:
                RAPID_TEMPLATE += line
        return RAPID_TEMPLATE

    def path2rapid_beta(self, path, module_name ='Robpath'):
        tool_name = 'tool' + module_name
        wobj_name = 'wobj' + module_name
        #TODO: Get powder and laser parameters
        feeder = 'gtv'
        laser = 'trudisk'

        if laser == 'rofin_rf':
            laser_out = 'Do_RF_ExterGate'
        elif laser == 'trudisk':
            laser_out = 'TdoPStartStat'

        laser_conf = self.rapid_laser_conf(laser)
        feeder_conf = self.rapid_feeder_conf(feeder)
        laser_stop = self.rapid_laser_stop(laser)
        feeder_stop = self.rapid_feeder_stop(feeder)

        RAPID_TEMPLATE = self.load_template()

        # Target points definition
        targets = ''
        for k in range(len(path)):
            p, q, b = path[k]
            targets = '\n'.join([targets, '    CONST robtarget Trobpath%i:=[[%f,%f,%f],[%f,%f,%f,%f],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];' %(k, p[0], p[1], p[2], q[3], q[0], q[1], q[2])])
        # Movement definition
        moves = ''
        laser_track = False
        laser_set = False
        for k in range(len(path)):
            p, q, process = path[k]
            if laser_track and process:
                if not laser_set:
                    moves = '\n'.join([moves, '    SetDO %s, 1;' % (laser_out)])
                    laser_set = True
                moves = '\n'.join([moves, '    MoveL Trobpath%i, vRobpath, z0, %s \WObj:=%s;' % (k, tool_name, wobj_name)])
            elif laser_track and not process:
                if laser_set:
                    laser_set = False
                    moves = '\n'.join([moves, '    MoveL Trobpath%i, vRobpath, z0, %s \WObj:=%s;' % (k, tool_name, wobj_name)])
                    moves = '\n'.join([moves, '    SetDO %s, 0;' % (laser_out)])
                else:
                    moves = '\n'.join([moves, '    TriggL Trobpath%i, vRobpath, laserON%s \T2:=laserOFF%s, z0, %s\WObj:=%s;' % (k, module_name, module_name, tool_name, wobj_name)])
            else:
                moves = '\n'.join([moves, '    MoveL Trobpath%i, vRobpathT, z0, %s \WObj:=%s;' % (k, tool_name, wobj_name)])
            laser_track = process
        moves = '\n'.join([moves, '\n'])

        tool = '[TRUE,[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]],[20,[70,30,123.5],[0,0,1,0],1,0,1]]' %(self.tool[0][0], self.tool[0][1], self.tool[0][2], self.tool[1][0], self.tool[1][1], self.tool[1][2], self.tool[1][3])
        wobj = '[FALSE,TRUE,"",[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]],[[0,0,0],[1,0,0,0]]]' %(self.workobject[0][0], self.workobject[0][1], self.workobject[0][2], self.workobject[1][0], self.workobject[1][1], self.workobject[1][2], self.workobject[1][3])

        return RAPID_TEMPLATE %{'module_name': module_name,
                                'laser_out': laser_out,
                                'laser_conf': laser_conf,
                                'laser_stop': laser_stop,
                                'feeder_conf':feeder_conf,
                                'feeder_stop':feeder_stop,
                                'tool': tool,
                                'tool_name': tool_name,
                                'wobj': wobj,
                                'wobj_name': wobj_name,
                                'speed': self.speed,
                                'speed_t': self.speed_t,
                                'targets': targets,
                                'moves': moves}

    def save_file(self, filename, routine):
        try:
            with open(filename, 'w') as f:
                f.writelines(routine)
        except IOError:
            pass

    def upload_file(self, filename, directory):
        try:
            ftp = ftplib.FTP(self.host, self.user, self.password, timeout=1)
            ftp.cwd(directory)
            ftp.storbinary('STOR ' + filename, open(filename,'rb'))
            print 'The file was upload sucessful!'
            print ftp.retrlines('LIST')
            ftp.quit()
        except IOError:
            print 'Upload file to robot error.'


if __name__ == '__main__':
    rob = Rapid()
    path = [([0, 0, 0], [0, 0, 0, 1], False),
            ([100, 0, 0], [0, 0, 0, 1], True),
            ([100, 100, 0], [0, 0, 0, 1], True)]
    filename = 'Output.mod'
    routine = rob.path2rapid_beta(path)
    rob.save_file(filename, routine)
    #rob.upload_file(filename, 'ETNA')
    print routine
