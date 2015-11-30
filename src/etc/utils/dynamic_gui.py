#!/usr/bin/env python

import wx
import wx.grid
import wx.lib.buttons, wx.lib.delayedresult
import sys, os, re, copy
import numpy
import threading
import textwrap

# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

import strategy
import project
import mapRenderer
from specCompiler import SpecCompiler



class MainGui(wx.Frame):
    def __init__(self, *args, **kwds):
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        
        self.layout()
        
        #load region file and display it
        self.compiler = SpecCompiler(sys.argv[1])
        self.proj = copy.deepcopy(self.compiler.proj)
        self.proj.rfi = self.proj.loadRegionFile(decomposed= False)
        
        self.Bind(wx.EVT_SIZE, self.onResize, self)
        self.region_map_window.Bind(wx.EVT_PAINT, self.draw_Map)
        
        #various initialisations
        self.env_buttons = []
        self.prev_region = None
        self.current_region = None
        self.invalid_regions = []
        self.sensorStates = {}
        self.region_map_window.SetBackgroundStyle(wx.BG_STYLE_CUSTOM)
        self.mapBitmap = None
        #listOfPropositions = self.findRnamePrevious + self.findRnameCurrent() + self.findSensors()
        
        self.displayRegionList()
        self.Bind(wx.EVT_LISTBOX_DCLICK, self.displayRegionDetails)
        self.region_map_window.Bind(wx.EVT_LEFT_DOWN, self.onMapClick)
        self.region_map_window.Bind(wx.EVT_PAINT, self.onPaint)
        
        self.populateToggleButtons(self.sizer_env, self.env_buttons, self.proj.all_sensors)
        
    
    def draw_Map(self, event):
        '''draws the map to the screen'''
        mapRenderer.drawMap(self.region_map_window, self.proj.rfi, scaleToFit=True)
        
        event.Skip()
    
    def onResize(self, event=None):
        size = self.region_map_window.GetSize()
        self.mapBitmap = wx.EmptyBitmap(size.x, size.y)
        if self.current_region is not None:
            hl = [self.current_region.name]
        else:
            hl = []

        self.mapScale = mapRenderer.drawMap(self.mapBitmap, self.proj.rfi, scaleToFit=True, drawLabels=True, memory=True,
                                            highlightList=hl, deemphasizeList=[r.name for r in self.invalid_regions])

        self.Refresh()
        self.Update()

        if event is not None:
            event.Skip()

    
    def displayRegionList(self):
        
        self.list_box_regions.Append(' ')
        for region in self.proj.rfi.regions:
            if not (region.isObstacle or region.name.lower() == "boundary"):
                self.list_box_regions.Append(region.name)
    
    def displayRegionDetails(self, event):
        
        for region in self.proj.rfi.regions:
            if (self.list_box_regions.GetString(self.list_box_regions.GetSelection())== region.name):
                
                #print dir(region)
                self.about_the_region.SetLabel("About the region: " + region.info)
                self.LTLstring.SetLabel("LTL Sring :"+ region.LTL)
                
                if (region.LTL != 'NONE'):
                        for button in self.env_buttons:
                            if (region.LTL.find(button.GetLabel()) != -1):
                                button.Enable(False)
                else:
                    for button in self.env_buttons:
                        button.Enable(True)

    def onMapClick(self, event):
        x = event.GetX()/self.mapScale
        y = event.GetY()/self.mapScale
        
        for region in self.proj.rfi.regions:
            if not (region.isObstacle or region.name.lower() == "boundary"):
                if (region.objectContainsPoint(x, y)):
                
                    self.prev_region = self.current_region
                    self.current_region = region
                    #need a way to determine invalid regions
                
                    self.old_region_name.SetLabel("Previous Region: " + self.prev_region.name)
                    self.present_region_name.SetLabel("Current Region: " + self.current_region.name)
                    self.about_the_region.SetLabel("About the region: " + region.info)
                    self.LTLstring.SetLabel("LTL Sring :"+ region.LTL)
                    
                    if (region.LTL != 'NONE'):
                        for button in self.env_buttons:
                            if (region.LTL.find(button.GetLabel()) != -1):
                                button.Enable(False)
                    else:
                        for button in self.env_buttons:
                            button.Enable(True)
                        
                #self.applySafetyConstraints()

        self.onResize() # Force map redraw
        event.Skip()
    
    
    def populateToggleButtons(self, target_sizer, button_container, button_names):
        for bn in button_names:
            # Create the new button and add it to the sizer
            name = textwrap.fill(bn, 100)
            button_container.append(wx.lib.buttons.GenToggleButton(self.window_pane_1, -1, name))
            target_sizer.Add(button_container[-1], 1, wx.EXPAND, 0)

            button_container[-1].SetFont(wx.Font(14, wx.DEFAULT, wx.NORMAL, wx.BOLD, 0, ""))

            self.window_pane_1.Layout() # Update the frame
            self.Refresh()
    
    def onPaint(self, event=None):
        if self.mapBitmap is None:
            return

        if event is None:
            dc = wx.ClientDC(self.region_map_window)
        else:
            pdc = wx.AutoBufferedPaintDC(self.region_map_window)
            try:
                dc = wx.GCDC(pdc)
            except:
                dc = pdc
            else:
                self.region_map_window.PrepareDC(pdc)

        dc.BeginDrawing()

        # Draw background
        dc.DrawBitmap(self.mapBitmap, 0, 0)

        
        dc.EndDrawing()

        if event is not None:
            event.Skip()
    
    ################## parsing functions################
    def findRnamePrevious(self):
        RnamePrevious = []
        for region in self.proj.rfi.regions:
            p = ['rob1_'+ regions]
            RnamePrevious = RnamePrevious + p
        return RnamePrevious

    def findRnameCurrent(self):
        RnameCurrent = []
        for region in self.proj.regions:
            c = ['rob_1_' + region + '_rc']
            RnameCurrent = RnameCurrent + c
        return RnameCurrent

    def findSensors(self):
        sensors =[]
        for name in self.proj.all_sensors:
            s = [name]
            sensors = sensors + s
        return sensors

    def parseBooleanFormulaRecurse(self, currentLine,APnames,boolString):
        operation = currentLine.pop(0)
    
        #print operation
        if operation=="":
            print >> sys.stderr, "Error reading the input string. Premature end of line."
        if operation=="|": 
            tmp1 = parseBooleanFormulaRecurse(currentLine,APnames,boolString)
            tmp2 = parseBooleanFormulaRecurse(currentLine,APnames,boolString)
            tmp = tmp1 or tmp2
            #print tmp1, tmp2, tmp
            return tmp
        if operation=="&":
            tmp1 = parseBooleanFormulaRecurse(currentLine,APnames,boolString)
            tmp2 = parseBooleanFormulaRecurse(currentLine,APnames,boolString)
            tmp = tmp1 and tmp2
            #print tmp1, tmp2, tmp
            return tmp
        if operation=="!":
            tmp = not parseBooleanFormulaRecurse(currentLine,APnames,boolString)
            #print tmp
            return tmp
        if operation=="1": 
            return True
        if operation=="0": 
            return False

        # Has to be a variable!
        for i,name in enumerate(APnames):
            if name==operation:
                #print name, boolString[i]
                return boolString[i]=='1'
        #print >> sys.stderr, "Error reading the input string. The variable "+operation+" has not been found."


    def parseBooleanFormula(self, currentLine,APnames,boolString):
        clSplit = currentLine.split(' ')
        #print clSplit
        result = parseBooleanFormulaRecurse(clSplit,APnames,boolString)
        return result
    
    #################layout ###########################
    def layout(self):   
        #defines the layout of the GUI
        
        self.window = wx.SplitterWindow(self, wx.ID_ANY, style=wx.SP_3D | wx.SP_BORDER)
        self.window_pane_1 = wx.Panel(self.window, wx.ID_ANY)
        self.window_pane_2 = wx.Panel(self.window, wx.ID_ANY)
        
        self.region_map_window = wx.Panel(self.window_pane_1, wx.ID_ANY, style=wx.SUNKEN_BORDER | wx.TAB_TRAVERSAL)
        #self.sensors_window = wx.Panel(self.window_pane_1, wx.ID_ANY, style = wx.SIMPLE_BORDER)
        self.list_box_regions = wx.ListBox(self.window_pane_2, wx.ID_ANY, style = wx.LB_SINGLE)
        
        self.region_map_label = wx.StaticText(self.region_map_window, wx.ID_ANY, "Region Map")
        self.sensors_label = wx.StaticText(self.window_pane_1, wx.ID_ANY, "Sensors")
        self.region_details_label = wx.StaticText(self.window_pane_2, wx.ID_ANY, "Region Details")
        self.list_box_regions_label = wx.StaticText(self.list_box_regions, wx.ID_ANY, "List of Regions")
        
        self.old_region_name = wx.StaticText(self.window_pane_2, wx.ID_ANY, "Previous Region: None")
        self.present_region_name = wx.StaticText(self.window_pane_2, wx.ID_ANY, "Current Region: None")
        #self.invalid_region_name = wx.StaticText(self.window_pane_2, wx.ID_ANY, "Invalid Region: None yet")
        self.about_the_region = wx.StaticText(self.window_pane_2, wx.ID_ANY, "About the region: ")
        self.LTLstring = wx.StaticText(self.window_pane_2, wx.ID_ANY, "LTL string:")
        
        self.SetTitle("Visualise Dynamics")
        self.SetSize((1170, 700))
        
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer_1 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_3 = wx.BoxSizer(wx.VERTICAL)
        sizer_env = wx.BoxSizer(wx.VERTICAL)
        
        
        sizer_1.Add(self.region_map_window, 2, wx.EXPAND, 1)
        sizer_1.Add(self.sensors_label, 2, wx.EXPAND, 1)
        #sizer_1.Add((20, 20), 0, 0, 0)
        sizer_1.Add(sizer_env, 1, wx.EXPAND, 0)
        
        self.window_pane_1.SetSizer(sizer_1)
        
        #region details stuff here instead of in a window
        sizer_3.Add(self.region_details_label, 0, 0, 0)
        sizer_3.Add((20, 20), 0, 0, 0)
        sizer_3.Add(self.old_region_name, 0, 0, 0)
        sizer_3.Add(self.present_region_name, 0, 0, 0)
        #sizer_3.Add(self.invalid_region_name, 0, 0, 0)
        sizer_3.Add((20, 20), 0, 0, 0)
        sizer_3.Add(self.about_the_region, 0, 0, 0)
        sizer_3.Add((20, 20), 0, 0, 0)
        sizer_3.Add(self.LTLstring, 0, 0, 0)
        
        #sizer_2.Add(self.region_details_window, 1, wx.EXPAND, 1)
        sizer_2.Add(sizer_3, 1, wx.EXPAND, 1)
        sizer_2.Add(self.list_box_regions, 1, wx.EXPAND, 1)
        
        self.window_pane_2.SetSizer(sizer_2)
        
        self.window.SplitHorizontally(self.window_pane_1, self.window_pane_2)
        sizer.Add(self.window, 1, wx.EXPAND, 0)
        self.SetSizer(sizer)
        self.Layout()
        
        self.sizer_env = sizer_env

if __name__ == "__main__":

    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    dynamics_GUI = MainGui(None, -1, "")
    app.SetTopWindow(dynamics_GUI)
    dynamics_GUI.Show()
    app.MainLoop()

