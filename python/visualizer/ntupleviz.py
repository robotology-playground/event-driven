# -*- coding: utf-8 -*-
"""
Copyright (C) 2020 Event-driven Perception for Robotics
Authors: Massimiliano Iacono
         Sim Bamford

This program is free software: you can redistribute it and/or modify it under 
the terms of the GNU General Public License as published by the Free Software 
Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY 
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
PARTICULAR PURPOSE.  See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with 
this program. If not, see <https://www.gnu.org/licenses/>.

Use kivy to create an app which can receive data dicts as imported by bimvee
importAe, and allow synchronised playback for each of the contained channels and datatypes. 
"""
# standard imports 
import matplotlib.pyplot as plt
import numpy as np
import sys, os
os.environ['KIVY_NO_ARGS'] = 'T'

# Optional import of tkinter allows setting of app size wrt screen size
try:
    import tkinter as tk
    from kivy.config import Config
    root = tk.Tk()
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    Config.set('graphics', 'position', 'custom')
    Config.set('graphics', 'left', int(screen_width/8))
    Config.set('graphics', 'top',  int(screen_width/8))
    Config.set('graphics', 'width',  int(screen_width/4*3))
    Config.set('graphics', 'height',  int(screen_height/4*3))
    #Config.set('graphics', 'fullscreen', 1)
except ModuleNotFoundError:
    pass

# kivy imports
from kivy.app import App
from kivy.uix.slider import Slider
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.uix.widget import Widget
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.textinput import TextInput
from kivy.uix.popup import Popup
from kivy.uix.button import Button
from kivy.uix.checkbox import CheckBox
from kivy.graphics.texture import Texture
from kivy.properties import ObjectProperty
from kivy.properties import StringProperty, NumericProperty, ListProperty, BooleanProperty
from kivy.properties import DictProperty, ReferenceListProperty
from kivy.metrics import dp

# To get the graphics, set this as the current working directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))
# local imports (from bimvee)
try:
    from visualiser import VisualiserDvs
    from visualiser import VisualiserFrame
    from visualiser import VisualiserPose6q
    from visualiser import VisualiserPoint3
    from timestamps import getLastTimestamp
except ModuleNotFoundError:
    if __package__ is None or __package__ == '':
        sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from libraries.bimvee.visualiser import VisualiserDvs
    from libraries.bimvee.visualiser import VisualiserFrame
    from libraries.bimvee.visualiser import VisualiserPose6q
    from libraries.bimvee.visualiser import VisualiserPoint3
    from libraries.bimvee.timestamps import getLastTimestamp
    
class ErrorPopup(Popup):
    label_text = StringProperty(None)


class WarningPopup(Popup):
    label_text = StringProperty(None)


class TextInputPopup(Popup):
    label_text = StringProperty(None)

class LoadDialog(FloatLayout):
    load = ObjectProperty(None)
    cancel = ObjectProperty(None)
    load_path = StringProperty(None)


class DictEditor(GridLayout):
    dict = DictProperty(None)

    def on_dict(self, instance, value):
        # 2020_03_10 Sim: Why only import Spinner here? at the top of the file 
        # it was causing a crash when starting in a thread - no idea why
        from kivy.uix.spinner import Spinner
        for n, topic in enumerate(sorted(value)):
            check_box = CheckBox()
            self.add_widget(check_box)
            self.add_widget(TextInput(text=str(n)))
            spinner = Spinner(values=['dvs', 'frame', 'pose6q', 'cam', 'imu'])
            if 'events' in topic:
                spinner.text = 'dvs'
                check_box.active = True
            elif 'image' in topic or 'depthmap' in topic:
                spinner.text = 'frame'
                check_box.active = True
            elif 'pose' in topic:
                spinner.text = 'pose6q'
                check_box.active = True

            self.add_widget(spinner)
            self.add_widget(TextInput(text=topic))

    def get_dict(self):
        from collections import defaultdict
        out_dict = defaultdict(dict)
        for dict_row in np.array(self.children[::-1]).reshape((-1, self.cols))[1:]:
            if not dict_row[0].active:
                continue
            ch = dict_row[1].text
            type = dict_row[2].text
            data = dict_row[3].text
            out_dict[ch][type] = data
        return out_dict

class TemplateDialog(FloatLayout):
    template = DictProperty(None)
    cancel = ObjectProperty(None)
    load = ObjectProperty(None)

class BoundingBox(Widget):
    def __init__(self, bb_color, x, y, width, height, **kwargs):
        super(BoundingBox, self).__init__(**kwargs)
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.bb_color = bb_color

class LabeledBoundingBox(BoundingBox):
    obj_label = StringProperty('')

    def __init__(self, bb_color, x, y, width, height, label, **kwargs):
        super(LabeledBoundingBox, self).__init__(bb_color, x, y, width, height, **kwargs)
        self.obj_label = '{:d}'.format(int(label))

class Viewer(BoxLayout):
    data = ObjectProperty(force_dispatch=True)
    need_init = BooleanProperty(True)
    dsm = ObjectProperty(None, allownone=True)
    flipHoriz = BooleanProperty(False)
    flipVert = BooleanProperty(False)
    colorfmt = 'luminance'

    def on_dsm(self, instance, value):
        if self.dsm is not None:
            self.colorfmt = self.dsm.get_colorfmt()
            x, y = self.dsm.get_dims()
            buf_shape = (dp(x), dp(y))
            self.image.texture = Texture.create(size=buf_shape, colorfmt=self.colorfmt)
            self.need_init = False
            
    def __init__(self, **kwargs):
        super(Viewer, self).__init__(**kwargs)
        #self.image = Image()
        #self.add_widget(self.image)
        self.cm = plt.get_cmap('tab20')
        self.image.texture = None
        Clock.schedule_once(self.init, 0)
        
    def init(self, dt):
        self.data = np.zeros((1, 1), dtype=np.uint8)

    def on_data(self, instance, value):
        self.update_data(self.data)

    def update(self):
        self.update_data(self.data)

    def update_data(self, data):
        if self.need_init:
            self.on_dsm(None, None)
        if self.image.texture is not None:
            x, y = self.dsm.get_dims()
            size_required = x * y * (1 + (self.colorfmt == 'rgb') * 2)
            if not isinstance(data, np.ndarray):
                data = np.zeros((x, y, 3), dtype=np.uint8)
            if data.size >= size_required:
                try:
                    if self.flipHoriz:
                        data = np.flip(data, axis = 1)
                    if not self.flipVert: # Not, because by default, y should increase downwards, following https://arxiv.org/pdf/1610.08336.pdf
                        data = np.flip(data, axis = 0)
                except AttributeError:
                    pass # It's not a class that allows flipping
                self.image.texture.blit_buffer(data.tostring(), bufferfmt="ubyte", colorfmt=self.colorfmt)

    def get_frame(self, time_value, time_window):
        pass


class LabelableViewer(Viewer):

    b_boxes = ObjectProperty(force_dispatch=True)
    b_boxes_visible = BooleanProperty(False)

    def get_b_boxes(self, time_value):
        self.b_boxes = self.dsm.get_b_box(time_value)

    def update(self):
        self.update_data(self.data)
        self.update_b_boxes(self.b_boxes, self.b_boxes_visible)

    def update_b_boxes(self, b_boxes, gt_visible=True):
        self.image.clear_widgets()

        if b_boxes is None:
            return
        if not gt_visible:
            return

        bb_copy = b_boxes.copy()
        texture_width = self.image.texture.width
        texture_height = self.image.texture.height
        image_width = self.image.norm_image_size[0]
        image_height = self.image.norm_image_size[1]

        x_img = self.image.center_x - image_width / 2
        y_img = self.image.center_y - image_height / 2

        w_ratio = image_width / texture_width
        h_ratio = image_height / texture_height
        for n, b in enumerate(bb_copy):
            for i in range(4):
                b[i] = dp(b[i])
            if self.flipHoriz:
                min_x = texture_width - b[3]
                max_x = texture_width - b[1]
                b[1] = min_x
                b[3] = max_x
            if self.flipVert:
                min_y = texture_height - b[2]
                max_y = texture_height - b[0]
                b[0] = min_y
                b[2] = max_y

            width = w_ratio * float(b[3] - b[1])
            height = h_ratio * float(b[2] - b[0])
            if width == 0 and height == 0:
                break

            x = x_img + w_ratio * float(b[1])
            y = y_img + h_ratio * (texture_height - float(b[2]))

            try:
                bb_color = self.cm.colors[b[4] % len(self.cm.colors)] + (1,)
                label = b[4]
                box_item = LabeledBoundingBox(id='box_{}'.format(n),
                                              bb_color=bb_color,
                                              x=x, y=y,
                                              width=width, height=height,
                                              label=label)
            except IndexError:
                box_item = BoundingBox(id='box_{}'.format(n),
                                       bb_color=self.cm.colors[0],
                                       x=x, y=y,
                                       width=width, height=height)
            self.image.add_widget(box_item)

    def on_b_boxes(self, instance, value):
        self.update_b_boxes(self.b_boxes, self.b_boxes_visible)


class ViewerDvs(Viewer):
    def __init__(self, **kwargs):
        super(ViewerDvs, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = plt.imread('graphics/missing.jpg')
        else:
            kwargs = {
                'polarised': self.polarised,
                'contrast': self.contrast,
                'pol_to_show': self.pol_to_show
                    }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)


class ViewerFrame(Viewer):
    def __init__(self, **kwargs):
        super(ViewerFrame, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = plt.imread('graphics/missing.jpg')
        else:
            kwargs = {
            }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)


class LabelableViewerDvs(LabelableViewer):
    def __init__(self, **kwargs):
        super(LabelableViewerDvs, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = plt.imread('graphics/missing.jpg')
        else:
            kwargs = {
                'polarised': self.polarised,
                'contrast': self.contrast,
                'pol_to_show': self.pol_to_show
                    }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)
        self.get_b_boxes(time_value)


class LabelableViewerFrame(LabelableViewer):
    def __init__(self, **kwargs):
        super(LabelableViewerFrame, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = plt.imread('graphics/missing.jpg')
        else:
            kwargs = {
                    }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)
        self.get_b_boxes(time_value)


class ViewerPose6q(Viewer):
    def __init__(self, **kwargs):
        super(ViewerPose6q, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = plt.imread('graphics/missing.jpg')
        else:
            kwargs = {
                'interpolate': self.interpolate,
                'perspective': self.perspective,
                    }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)
   
class ViewerPoint3(Viewer):
    def __init__(self, **kwargs):
        super(ViewerPoint3, self).__init__(**kwargs)

    def get_frame(self, time_value, time_window):
        if self.dsm is None:
            self.data = plt.imread('graphics/missing.jpg')
        else:
            kwargs = {
                'perspective': self.perspective,
                'yaw': self.yaw,
                'pitch': self.pitch,
                    }
            self.data = self.dsm.get_frame(time_value, time_window, **kwargs)
   
class DataController(GridLayout):
    ending_time = NumericProperty(.0)
    filePathOrName = StringProperty('')
    data_dict = DictProperty({}) # A bimvee-style container of channels
    
    def __init__(self, **kwargs):
        super(DataController, self).__init__(**kwargs)

    def update_children(self):
        for child in self.children:
            child.get_frame(self.time_value, self.time_window)
       
    def add_viewer_and_resize(self, data_type, data_dict, label='', with_boxes=False):
        if data_type == 'dvs':
            new_viewer = LabelableViewerDvs() if with_boxes else ViewerDvs()
            visualiser = VisualiserDvs(data_dict)
        elif data_type == 'frame':
            new_viewer = LabelableViewerFrame() if with_boxes else ViewerFrame()
            visualiser = VisualiserFrame(data_dict)
        elif data_type == 'pose6q':
            new_viewer = ViewerPose6q()
            visualiser = VisualiserPose6q(data_dict)
            label = 'red=x green=y, blue=z ' + label
        elif data_type == 'point3':
            new_viewer = ViewerPoint3()
            visualiser = VisualiserPoint3(data_dict)
        new_viewer.label = label
        new_viewer.dsm = visualiser
        self.add_widget(new_viewer)

        self.cols = int(np.ceil(np.sqrt(len(self.children))))

    def add_viewer_for_each_channel_and_data_type(self, in_dict, label='', recursionDepth=0):
        if isinstance(in_dict, list):
            print('    ' * recursionDepth + 'Received a list - looking through the list for containers...')
            for num, in_dict_element in enumerate(in_dict):
                self.add_viewer_for_each_channel_and_data_type(in_dict_element, label=label+':'+str(num), recursionDepth=recursionDepth+1)
        elif isinstance(in_dict, dict):
            print('    ' * recursionDepth + 'Received a dict - looking through its keys ...')
            for key_name in in_dict.keys():
                print('    ' * recursionDepth + 'Dict contains a key "' + key_name + '" ...')
                if isinstance(in_dict[key_name], dict):
                    if 'ts' in in_dict[key_name]:
                        if key_name in ['dvs', 'frame', 'pose6q']:
                            print('    ' * recursionDepth + 'Creating a new viewer, of type: ' + key_name)
                            self.add_viewer_and_resize(key_name, in_dict[key_name], label=label+':'+str(key_name),
                                                       with_boxes='b_boxes' in in_dict[key_name].keys())
                        else:
                            print('    ' * recursionDepth + 'Datatype not supported: ' + key_name)
                    else: # recurse through the sub-dict
                        self.add_viewer_for_each_channel_and_data_type(in_dict[key_name], label=label+':'+str(key_name), recursionDepth=recursionDepth+1)
                elif isinstance(in_dict[key_name], list):
                    self.add_viewer_for_each_channel_and_data_type(in_dict[key_name], label=label+':'+str(key_name), recursionDepth=recursionDepth+1)
                    
                else:
                    print('    ' * recursionDepth + 'Ignoring that key ...')


    
    def on_data_dict(self, instance, value):
        self.ending_time = float(getLastTimestamp(self.data_dict)) # timer is watching this
        while len(self.children) > 0:
            self.remove_widget(self.children[0])
            print('Removed an old viewer; num remaining viewers: ' + str(len(self.children)))
        self.add_viewer_for_each_channel_and_data_type(self.data_dict)

    def dismiss_popup(self): 
        if hasattr(self, '_popup'):
            self._popup.dismiss()

    def show_template_dialog(self, topics):
        self.dismiss_popup()
        content = TemplateDialog(template=topics,
                                 cancel=self.dismiss_popup,
                                 load=self.load)
        self._popup = Popup(title="Define template", content=content,
                            size_hint=(0.9, 0.9))
        self._popup.open()
                

    def show_load(self):
        self.dismiss_popup()
        # FOR DEBUGGING
        self.load('/media/miacono/Shared/datasets/ball/numpy', ['events.npy'])
        return
        content = LoadDialog(load=self.load,
                             cancel=self.dismiss_popup)
        self._popup = Popup(title="Load file", content=content,
                            size_hint=(0.9, 0.9))
        self._popup.open()

    def load(self, path, selection, template=None):
        self.dismiss_popup()
        try:
            from importAe import importAe
        except ModuleNotFoundError:
            from libraries.bimvee.importAe import importAe

        from os.path import join

        # If both path and selection are None than it will try to reload previously given path
        if path is not None or selection is not None:
            if selection:
                self.filePathOrName=join(path, selection[0])
            else:
                self.filePathOrName=path

        try:
            self.data_dict = importAe(filePathOrName=self.filePathOrName, template=template)
        except ValueError:
            try:
                from importRosbag import importRosbag
            except ModuleNotFoundError:
                from libraries.bimvee.importRosbagSubmodule.importRosbag import importRosbag
            topics = importRosbag(filePathOrName=self.filePathOrName, listTopics=True)
            self.show_template_dialog(topics)

        self.update_children()

#    def dismiss_popup(self):
#        if self._popup is not None:
#            self._popup.dismiss()
    
class TimeSlider(Slider):
    def __init__(self, **kwargs):
        super(TimeSlider, self).__init__(**kwargs)
        self.clock = None
        self.speed = 1
    
    def increase_slider(self, dt):
        self.value = min(self.value + dt / self.speed, self.max)
        if self.value >= self.max:
            if self.clock is not None:
                self.clock.cancel()
    
    def decrease_slider(self, dt):
        self.value = max(self.value - dt / self.speed, 0.0)
        if self.value <= 0.0:
            if self.clock is not None:
                self.clock.cancel()
    
    def play_pause(self):
        if self.clock is None:
            self.clock = Clock.schedule_interval(self.increase_slider, 0.001)
        else:
            if self.clock.is_triggered:
                self.clock.cancel()
            else:
                self.clock.cancel()
                self.clock = Clock.schedule_interval(self.increase_slider, 0.001)

    def pause(self):
        if self.clock is not None:
            self.clock.cancel() 
 
    def play_forward(self):
        if self.clock is not None:
            self.clock.cancel() 
        self.clock = Clock.schedule_interval(self.increase_slider, 0.001)
 
    def play_backward(self):
        if self.clock is not None:
            self.clock.cancel() 
        self.clock = Clock.schedule_interval(self.decrease_slider, 0.001)
 
    def stop(self):
        if self.clock is not None:
            self.clock.cancel()
            self.set_norm_value(0)
    
    def reset(self):
        self.value = 0

    def step_forward(self):
        #self.increase_slider(self.time_window)
        self.increase_slider(0.016)

    def step_backward(self):
        #self.decrease_slider(self.time_window)
        self.decrease_slider(0.016)
                
class RootWidget(BoxLayout):
    def __init__(self, **kwargs):
        super(RootWidget, self).__init__(**kwargs)


class Ntupleviz(App):
    def build(self):
        return RootWidget()

if __name__ == '__main__':
    Ntupleviz().run()

