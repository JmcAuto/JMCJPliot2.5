#!/usr/bin/env python

###############################################################################
# Copyright 2017 The JmcAuto Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import sys
sys.path.append("/home/tmp/ros/lib/python2.7/dist-packages/")
import sys
import gflags
import os

import modules.map.proto.map_pb2 as map_pb2
import modules.routing.proto.topo_graph_pb2 as topo_graph_pb2
import modules.routing.proto.routing_pb2 as routing_pb2
from google.protobuf import text_format
import matplotlib.pyplot as plt

FLAGS = gflags.FLAGS
gflags.DEFINE_string('map_dir', 'modules/map/data/demo', 'map directory')


def get_map_dir(argv):
    """

    """
    sys.argv.insert(1, '--undefok')
    flagfile = os.path.normpath(
        os.path.join(
            os.path.dirname(__file__), '../../common/data',
            'global_flagfile.txt'))
    sys.argv.insert(2, '--flagfile=' + flagfile)
    argv = FLAGS(sys.argv)
    mapdir = os.path.normpath(
        os.path.join(os.path.dirname(__file__), '../../../', FLAGS.map_dir))
    print "Map dir: ", FLAGS.map_dir
    return mapdir


def get_mapdata(map_dir):
    print 'Please wait for loading map data...'
    map_data_path = os.path.join(map_dir, 'base_map.bin')
    print "file: ", map_data_path
    f_handle = open(map_data_path)
    base_map = map_pb2.Map()
    base_map.ParseFromString(f_handle.read())
    f_handle.close()
    print 'Done'
    return base_map


def get_topodata(map_dir):
    print 'Please wait for loading routing topo data...'
    topo_data_path = os.path.join(map_dir, 'routing_map.bin')
    print "file: ", topo_data_path
    f_handle = open(topo_data_path)
    graph = topo_graph_pb2.Graph()
    graph.ParseFromString(f_handle.read())
    f_handle.close()
    print 'Done'
    return graph


def get_routingdata():
    print 'Please wait for loading route response data...'
    log_dir = os.path.normpath(
        os.path.join(os.path.dirname(__file__), '../../../data/log'))
    route_data_path = os.path.join(log_dir, 'passage_region_debug.bin')
    print "file: ", route_data_path
    f_handle = open(route_data_path)
    route = routing_pb2.RoutingResponse()
    text_format.Parse(f_handle.read(), route)
    f_handle.close()
    print "Done"
    return route


def onclick(event):
    """Event function when mouse left button is clicked"""
    print '\nClick captured! x=%f\ty=%f' % (event.xdata, event.ydata)
    print 'cmd>',


def downsample_array(array, step=5):
    """down sample given array"""
    result = array[::step]
    result.append(array[-1])
    return result


def draw_boundary(ax, line_segment):
    """
    :param line_segment:
    :return:
    """
    px = []
    py = []
    for p in line_segment.point:
        px.append(float(p.x))
        py.append(float(p.y))
    px = downsample_array(px)
    py = downsample_array(py)
    ax.plot(px, py, 'k', lw=0.4)


def draw_map(ax, mapfile):
    """ draw map from mapfile"""

    for lane in mapfile.lane:
        for curve in lane.left_boundary.curve.segment:
            if curve.HasField('line_segment'):
                draw_boundary(ax, curve.line_segment)

        for curve in lane.right_boundary.curve.segment:
            if curve.HasField('line_segment'):
                draw_boundary(ax, curve.line_segment)
    plt.draw()
