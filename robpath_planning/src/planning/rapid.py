import math
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

        self.tool = [[227.49,-23.1497,79.8447], [0.70711, 0, -0.70711, 0]] # Tool pose
        #self.tool = [[351.1,-36.6,86.9],[-0.5000, -0.0000, 0.8660, -0.0000]] # Tool pose 60
        self.workobject = [[-145.911,-202.236,13.7597], [0.999938,-0.0000854863,0.000281991,0.0110406]] # Work Object pose
        #self.workobject = [[1255, -87, 1032], [0.999939,0.00523022,0.000667568,-0.00966806]] # Work Object pose 60

        self.offset = 5
        self.offset_z = 5
        self.start_lag = 0.2

        self.laser_type = 'trudisk'
        self.feeder_type = 'gtv'
        
        self.dynamic_params = []
        self.params_group = []

    def set_process(self, speed, power, travel=50):
        self.speed = speed
        self.power = power
        self.speed_t = travel

    def set_powder(self, carrier, stirrer, turntable):
        self.carrier = carrier
        self.stirrer = stirrer
        self.turntable = turntable

    def rapid_laser_conf(self):
        LASER_TEMPLATE = ''
        if self.laser_type == 'rofin_rf':
            LASER_TEMPLATE += 'Set Do_RF_MainOn;\n'
            LASER_TEMPLATE += '    Set Do_RF_StandByOn;\n'
            LASER_TEMPLATE += '    WaitDI DI_RF_LaserBeamReady,1;\n'
            LASER_TEMPLATE += '    WaitDI DI_RF_GeneralFault,0;\n'
            LASER_TEMPLATE += '\n'
            LASER_TEMPLATE += '    SetGO GO_Program_Rf, 0;\n' # set the program for control of laser power - prog 5
            LASER_TEMPLATE += '    WaitTime 1;\n'
            LASER_TEMPLATE += '    !SetGO GoLDL_Pwr3, %(power)i;\n' # set the laser power - 2200 W
        elif self.laser_type == 'trudisk':
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
        elif self.laser_type == 'waam':
            LASER_TEMPLATE += '\n! Sin inicializacion laser (waam)\n'
        return LASER_TEMPLATE %{'power': self.power}

    def rapid_laser_stop(self):
        LASER_TEMPLATE = ''
        if self.laser_type == 'rofin_rf':
            LASER_TEMPLATE += 'Reset Do_RF_StandByOn;\n'
        elif self.laser_type == 'trudisk':
            LASER_TEMPLATE += 'Reset TdoActLaser;\n'
            LASER_TEMPLATE += '    Reset TdoStandBy;\n'
            LASER_TEMPLATE += '    Reset TdoLaserOn;\n'
            LASER_TEMPLATE += '    Reset TdoExtActiv;'
        elif self.laser_type == 'waam':
            LASER_TEMPLATE += '\n! Sin parada laser (waam)\n'
        return LASER_TEMPLATE

    def rapid_feeder_conf(self, module_name='Robpath'):
        FEEDER_TEMPLATE = '    Set DoRootGas;\n'
        if self.feeder_type == 'tps5000':
            FEEDER_TEMPLATE += 'Set DoCossJet;\n'
            FEEDER_TEMPLATE += 'PulseDO\PLength:=0.5,doTPSReset;\n'
            FEEDER_TEMPLATE += 'Set doTPSReady;\n'
            FEEDER_TEMPLATE += 'Set doTPSOP0;\n'
            FEEDER_TEMPLATE += 'Reset doTPSOP1;\n'
            FEEDER_TEMPLATE += 'Set doTPSOP2;\n'
            FEEDER_TEMPLATE += '!SetGO GoTPSJobL, 100;\n'
            FEEDER_TEMPLATE += 'TriggEquip wireON'+module_name+', 0 \Start, 0 \DOp:=doTPSWireF, 1;\n'
            FEEDER_TEMPLATE += 'TriggEquip wireOFF'+module_name+', 0, 0 \DOp:=doTPSWireF, 0;\n'
        elif self.feeder_type == 'medicoat':
            FEEDER_TEMPLATE += '    !MedicoatL2 "OFF", 5, 20, 7.5;\n' # gas de arrastre, stirrer, turntable
            FEEDER_TEMPLATE += '    MedicoatL1 "OFF", %(carrier)i, %(stirrer)i, %(turntable)i;'
        elif self.feeder_type == 'gtv':
            FEEDER_TEMPLATE += '    SetAO AoGTV_ExternDisk, 35;\n'
            FEEDER_TEMPLATE += '    SetAO AoGTV_ExternMassFlow, 26.6;\n'
            FEEDER_TEMPLATE += '    Set doGTV_StartExtern;\n'
            FEEDER_TEMPLATE += '    WaitTime 15;'
        elif self.feeder_type == 'tps5000waam':
            FEEDER_TEMPLATE += '\nSet doTPSReady;\nSet doFr1RobotReady;\nSet doFr1ErrorReset;\nSet doTPSReset;\nSetGo goFr1Mode,1;\nReset doTPSOP0;\nSet doTPSOP1;\nReset doTPSOP2;\nSet doFr1WeldingSim;\n'
            FEEDER_TEMPLATE += 'TriggEquip wireON'+module_name+', 2 \Start, 0 \DOp:=doFr1ArcOn, 1;\n'
            FEEDER_TEMPLATE += 'TriggEquip wireOFF'+module_name+', 2, 0 \DOp:=doFr1ArcOn, 0;\n'
        return FEEDER_TEMPLATE %{'carrier': self.carrier,
                                'stirrer': self.stirrer,
                                'turntable': self.turntable}

    def rapid_feeder_stop(self):
        FEEDER_TEMPLATE = ''
        if self.feeder_type == 'medicoat':
            FEEDER_TEMPLATE += 'Reset doMdtPL2On;\n'
            FEEDER_TEMPLATE += 'Reset doMdtPL1On;\n'
            FEEDER_TEMPLATE += '    Reset DoRootGas;\n'
        elif self.feeder_type == 'gtv':
            FEEDER_TEMPLATE += 'Reset doGTV_StartExtern;\n'
            FEEDER_TEMPLATE += '    Reset DoRootGas;\n'
        elif self.feeder_type == 'tps5000':
            FEEDER_TEMPLATE += '    Reset DoCossJet;\n'
            FEEDER_TEMPLATE += '    Reset DoRootGas;\n'
        elif self.feeder_type == 'tps5000waam':
            FEEDER_TEMPLATE += '\nReset doTPSReady;\n'
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

    def path2rapid_beta_old(self, path, module_name='Robpath'):
        tool_name = 'tool' + module_name
        wobj_name = 'wobj' + module_name
        speed_name = 'vRobpath'
        speed_t_name = 'vRobpathT'
        # TODO: Get powder and laser parameters

        if self.laser_type == 'rofin_rf':
            laser_out = 'Do_RF_ExterGate'
        elif self.laser_type == 'trudisk':
            laser_out = 'TdoPStartStat'
        elif self.laser_type == 'waam':
            laser_out = 'doFr1ArcOn'

        laser_conf = self.rapid_laser_conf()
        feeder_conf = self.rapid_feeder_conf(module_name)
        laser_stop = self.rapid_laser_stop()
        feeder_stop = self.rapid_feeder_stop()

        RAPID_TEMPLATE = self.load_template()

        feeder_triggs = ''
        if self.feeder_type == 'tps5000' or self.feeder_type == 'tps5000waam':
            feeder_triggs = '\n'.join([feeder_triggs, 'VAR triggdata wireON%s;' % (module_name)])
            feeder_triggs = '\n'.join([feeder_triggs, 'VAR triggdata wireOFF%s;' % (module_name)])

        # Target points definition
        targets = ''
        for k in range(len(path)):
            p, q, b = path[k]
            targets = '\n'.join([targets, '    CONST robtarget Trobpath%i:=[[%f,%f,%f],[%f,%f,%f,%f],[0,0,0,0],[ext_axis0,ext_axis1,ext_axis2,9E+09,9E+09,9E+09]];' %(k, p[0], p[1], p[2], q[3], q[0], q[1], q[2])])
        # Movement definition
        moves = ''
        laser_track = False
        laser_set = False
        return_track = False
        p_ant, q_ant, process_ant = None, None, None
        self.offset_x = 0
        self.offset_y = 0
        for k in range(len(path)):
            p, q, process = path[k]
            if len(path) == len(self.params_group):
                if self.params_group[k] != -1:
                    speed_name = '[%s,500,5000,1000]' % (self.dynamic_params[self.params_group[k]]['speed'])
            if return_track:
                #TODO: revisar se se poderia facer esto cando se move ao punto inicial
                if p_ant is not None:
                    if p_ant[2] < p[2]:
                        # Slice identifier based on Z increment
                        moves = '\n'.join([moves, '!SLICE at %f mm' % (p[2])])
                # if self.feeder_type == 'tps5000':
                #     moves = '\n'.join([moves, '    MoveL Offs(Trobpath%i, %f, %f, %f), vRobpathT, z0, %s \WObj:=%s;' % (k, -1 * self.offset_x, -1 * self.offset_y, (self.offset_z + 15), tool_name, wobj_name)])
                if self.feeder_type == 'tps5000' or self.feeder_type == 'tps5000waam':
                    # In wire process, approach to point with offsets
                    #TODO: cambiar a condicion do IF para que sexa sempre que hai offsets
                    moves = '\n'.join([moves, '    MoveL Offs(Trobpath%i, %f, %f, %f), vRobpathT, z0, %s \WObj:=%s;' % (k, -1 * self.offset_x, -1 * self.offset_y, self.offset_z, tool_name, wobj_name)])
                else:
                    pass
                    #TODO: Este else sobra
                    #moves = '\n'.join([moves, '    MoveL Trobpath%i, vRobpathT, z0, %s \WObj:=%s;' % (k, tool_name, wobj_name)])
                return_track = False
            if laser_track and process:
                # Point inside a track, multiline track
                if self.feeder_type == 'tps5000waam':
                    # Triggers only for process ON
                    moves = '\n'.join([moves, '    TriggL Trobpath%i, %s, laserON%s, z0, %s \WObj:=%s;' % (k, speed_name, module_name, tool_name, wobj_name)])
                else:
                    # Set process signal and moveL
                    if not laser_set:
                        moves = '\n'.join([moves, '    SetDO %s, 1;' % (laser_out)])
                        if self.feeder_type == 'tps5000':
                            moves = '\n'.join([moves, '    SetDO doTPSWireF, 1;'])
                            moves = '\n'.join([moves, '    WaitTime %f;' % (self.start_lag)])
                        laser_set = True
                    moves = '\n'.join([moves, '    MoveL Trobpath%i, %s, z0, %s \WObj:=%s;' % (k, speed_name, tool_name, wobj_name)])
            elif laser_track and not process:
                # Last point of a track
                if laser_set:
                    # Last point of a multiline track
                    laser_set = False
                    if self.feeder_type == 'tps5000waam':
                        # Trigger ending with process OFF
                        moves = '\n'.join([moves, '    TriggL Trobpath%i, %s, laserOFF%s, fine, %s \WObj:=%s;' % (k, speed_name, module_name, tool_name, wobj_name)])
                    else:
                        # Move to point and OFF process
                        moves = '\n'.join([moves, '    MoveL Trobpath%i, %s, fine, %s \WObj:=%s;' % (k, speed_name, tool_name, wobj_name)])
                        if self.feeder_type == 'tps5000':
                            moves = '\n'.join([moves, '    SetDO doTPSWireF, 0;'])
                        else:
                            moves = '\n'.join([moves, '    SetDO %s, 0;' % (laser_out)])
                else:
                    # Last point of a single line track
                    if self.feeder_type == 'tps5000':
                        moves = '\n'.join([moves, '    SetDO %s, 1;' % (laser_out)])
                        moves = '\n'.join([moves, '    SetDO doTPSWireF, 1;'])
                        moves = '\n'.join([moves, '    WaitTime %f;' % (self.start_lag)])
                        moves = '\n'.join([moves, '    MoveL Trobpath%i, %s, fine, %s\WObj:=%s;' % (k, speed_name, tool_name, wobj_name)])
                        moves = '\n'.join([moves, '    SetDO doTPSWireF, 0;'])
                        #moves = '\n'.join([moves, '    TriggL Trobpath%i, vRobpath, laserON%s \T2:=laserOFF%s \T3:=wireON%s \T4:=wireOFF%s, z0, %s\WObj:=%s;' % (k, module_name, module_name, module_name, module_name, tool_name, wobj_name)])
                    else:
                        moves = '\n'.join([moves, '    TriggL Trobpath%i, %s, laserON%s \T2:=laserOFF%s, fine, %s\WObj:=%s;' % (k, speed_name, module_name, module_name, tool_name, wobj_name)])
                    if self.offset > 0 or self.offset_z > 0:
                        # Add offset point if necessary
                        delta_x = p[0] - p_ant[0]
                        delta_y = p[1] - p_ant[1]
                        delta_t = math.sqrt(delta_x**2 + delta_y**2)
                        self.offset_x = self.offset * delta_x / delta_t
                        self.offset_y = self.offset * delta_y / delta_t
                        if self.feeder_type == 'tps5000':
                            moves = '\n'.join([moves, '    MoveL Offs(Trobpath%i, %f, %f, %f), vRobpathT, fine, %s \WObj:=%s;' % (k, self.offset_x, self.offset_y, self.offset_z, tool_name, wobj_name)])
                            moves = '\n'.join([moves, '    SetDO %s, 0;' % (laser_out)])
                            moves = '\n'.join([moves, '    MoveL Offs(Trobpath%i, %f, %f, %f), vRobpathT, z0, %s \WObj:=%s;' % (k, self.offset_x, self.offset_y, (self.offset_z + 15), tool_name, wobj_name)])
                        elif self.feeder_type == 'tps5000waam':
                            moves = '\n'.join([moves, '    MoveL Offs(Trobpath%i, %f, %f, %f), vRobpathT, z0, %s \WObj:=%s;' % (k, self.offset_x, self.offset_y, self.offset_z, tool_name, wobj_name)])
                        else:
                            # moves = '\n'.join([moves, '    MoveL Trobpath%i, vRobpathT, z0, %s \WObj:=%s;' % (k, tool_name, wobj_name)])
                            moves = '\n'.join([moves, '    MoveL Offs(Trobpath%i, %f, %f, %f), vRobpathT, z0, %s \WObj:=%s;' % (k, self.offset_x, self.offset_y, self.offset_z, tool_name, wobj_name)])
                            #moves = '\n'.join([moves, '    MoveL Offs(Trobpath%i, -30, 30, 0), vRobpathT, z0, %s \WObj:=%s;' % (k, tool_name, wobj_name)])
                    else:
                        #TODO: Creo que non fai falta, revisar
                        if self.feeder_type == 'tps5000' or self.feeder_type == 'tps5000waam':
                            moves = '\n'.join([moves, '    SetDO %s, 0;' % (laser_out)])
                    return_track = True
            elif not laser_track and process:
                # Initial point of a track
                moves = '\n'.join([moves, '    MoveL Trobpath%i, vRobpathT, fine, %s \WObj:=%s;' % (k, tool_name, wobj_name)])
            else:
                # Multiline travel movement
                moves = '\n'.join([moves, '    MoveL Trobpath%i, vRobpathT, z0, %s \WObj:=%s;' % (k, tool_name, wobj_name)])
            # Update last point
            laser_track = process # Same as process_ant
            p_ant, q_ant, process_ant = path[k]
        moves = '\n'.join([moves, '\n'])

        tool = '[TRUE,[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]],[20,[70,30,123.5],[0,0,1,0],1,0,1]]' %(self.tool[0][0], self.tool[0][1], self.tool[0][2], self.tool[1][0], self.tool[1][1], self.tool[1][2], self.tool[1][3])
        wobj = '[FALSE,FALSE,"STN1",[[0,0,0],[1,0,0,0]],[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]]]' %(self.workobject[0][0], self.workobject[0][1], self.workobject[0][2], self.workobject[1][0], self.workobject[1][1], self.workobject[1][2], self.workobject[1][3])

        return RAPID_TEMPLATE %{'module_name': module_name,
                                'laser_out': laser_out,
                                'laser_conf': laser_conf,
                                'laser_stop': laser_stop,
                                'feeder_conf': feeder_conf,
                                'feeder_stop': feeder_stop,
                                'feeder_triggs': feeder_triggs,
                                'tool': tool,
                                'tool_name': tool_name,
                                'wobj': wobj,
                                'wobj_name': wobj_name,
                                'speed': self.speed,
                                'speed_t': self.speed_t,
                                'targets': targets,
                                'moves': moves}

    def first_process_point(self):
        #TODO
        pass

    def path2rapid_beta(self, path, module_name='Robpath'):
        tool_name = 'tool' + module_name
        wobj_name = 'wobj' + module_name
        speed_name = 'vRobpath'
        speed_t_name = 'vRobpathT'
        # TODO: Get powder and laser parameters

        if self.laser_type == 'rofin_rf':
            laser_out = 'Do_RF_ExterGate'
        elif self.laser_type == 'trudisk':
            laser_out = 'TdoPStartStat'
        elif self.laser_type == 'waam':
            laser_out = 'doFr1ArcOn'

        laser_conf = self.rapid_laser_conf()
        feeder_conf = self.rapid_feeder_conf(module_name)
        laser_stop = self.rapid_laser_stop()
        feeder_stop = self.rapid_feeder_stop()

        RAPID_TEMPLATE = self.load_template()

        feeder_triggs = ''
        if self.feeder_type == 'tps5000' or self.feeder_type == 'tps5000waam':
            feeder_triggs = '\n'.join([feeder_triggs, 'VAR triggdata wireON%s;' % (module_name)])
            feeder_triggs = '\n'.join([feeder_triggs, 'VAR triggdata wireOFF%s;' % (module_name)])

        # Target points definition
        targets = ''
        n_targets = 0
        # Movement definition
        moves = ''
        self.offset_x = 0
        self.offset_y = 0
        for l, layer in enumerate(path):
            moves = '\n'.join([moves, '!SLICE %i' % (l)])
            for t, track in enumerate(layer):
                moves = '\n'.join([moves, '!TRACK %i' % (t)])
                targets = '\n'.join([targets, '!TRACK %i' % (t)])
                for np, point in enumerate(track):
                    p, q = point
                    targets = '\n'.join([targets, '    CONST robtarget Trobpath%i:=[[%f,%f,%f],[%f,%f,%f,%f],[0,0,0,0],[ext_axis0,ext_axis1,ext_axis2,9E+09,9E+09,9E+09]];' %(n_targets, p[0], p[1], p[2], q[3], q[0], q[1], q[2])])
                    if np == 0:
                        #Initial point of a track
                        if self.offset_z != 0 or self.offset != 0:
                            delta_x = p[0] - track[np+1][0][0]
                            delta_y = p[1] - track[np+1][0][1]
                            delta_t = math.sqrt(delta_x**2 + delta_y**2)
                            self.offset_x = self.offset * delta_x / delta_t
                            self.offset_y = self.offset * delta_y / delta_t
                            moves = '\n'.join([moves, '    MoveL Offs(Trobpath%i, %f, %f, %f), vRobpathT, z0, %s \WObj:=%s;' % (n_targets, self.offset_x, self.offset_y, self.offset_z, tool_name, wobj_name)])
                        moves = '\n'.join([moves, '    MoveL Trobpath%i, vRobpathT, fine, %s \WObj:=%s;' % (n_targets, tool_name, wobj_name)])
                        moves = '\n'.join([moves, '    SetDO %s, 1;' % (laser_out)])
                    elif np == len(track)-1:
                        #Last point of a track
                        moves = '\n'.join([moves, '    MoveL Trobpath%i, %s, fine, %s \WObj:=%s;' % (n_targets, speed_name, tool_name, wobj_name)])
                        moves = '\n'.join([moves, '    SetDO %s, 0;' % (laser_out)])
                        if self.offset_z != 0 or self.offset != 0:
                            delta_x = p[0] - track[np-1][0][0]
                            delta_y = p[1] - track[np-1][0][1]
                            delta_t = math.sqrt(delta_x**2 + delta_y**2)
                            self.offset_x = self.offset * delta_x / delta_t
                            self.offset_y = self.offset * delta_y / delta_t
                            moves = '\n'.join([moves, '    MoveL Offs(Trobpath%i, %f, %f, %f), vRobpathT, z0, %s \WObj:=%s;' % (n_targets, self.offset_x, self.offset_y, self.offset_z, tool_name, wobj_name)])
                    else:
                        #Intermediate point
                        moves = '\n'.join([moves, '    MoveL Trobpath%i, %s, z0, %s \WObj:=%s;' % (n_targets, speed_name, tool_name, wobj_name)])
                    n_targets += 1
        moves = '\n'.join([moves, '\n'])

        tool = '[TRUE,[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]],[20,[70,30,123.5],[0,0,1,0],1,0,1]]' %(self.tool[0][0], self.tool[0][1], self.tool[0][2], self.tool[1][0], self.tool[1][1], self.tool[1][2], self.tool[1][3])
        wobj = '[FALSE,FALSE,"STN1",[[0,0,0],[1,0,0,0]],[[%.1f,%.1f,%.1f],[%f,%f,%f,%f]]]' %(self.workobject[0][0], self.workobject[0][1], self.workobject[0][2], self.workobject[1][0], self.workobject[1][1], self.workobject[1][2], self.workobject[1][3])

        return RAPID_TEMPLATE %{'module_name': module_name,
                                'laser_out': laser_out,
                                'laser_conf': laser_conf,
                                'laser_stop': laser_stop,
                                'feeder_conf': feeder_conf,
                                'feeder_stop': feeder_stop,
                                'feeder_triggs': feeder_triggs,
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
