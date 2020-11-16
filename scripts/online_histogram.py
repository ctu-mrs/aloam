#!/usr/bin/env python

import rospy
import sys, random

import numpy
import matplotlib.pyplot as plt
from scipy.stats import norm
from aloam_slam.msg import AloamDiagnostics, FeatureSelectionDiagnostics

class OnlineHist():
    def __init__(self):
        self.has_new_data = False

        rospy.init_node('online_histogram', anonymous=True)
        self.subscriber = rospy.Subscriber("/uav68/aloam/diag", AloamDiagnostics, self.callback_diag, queue_size=1)

        self.BINS = 50
        figsize=(10, 8)
        # self.fig = plt.figure(num=None, figsize=figsize, dpi=40, facecolor='w', edgecolor='k')
        # self.ax = self.fig.add_sublot(111)
        self.fig, self.axs = plt.subplots(2, 3, figsize=figsize)

        plt.ion()
        plt.show()
        # plt.show(block=False)
        # plt.show(True)
        # ax.set_xlabel('x [m]',labelpad=20, fontsize=xy_label_font_size)
        # ax.set_ylabel('y [m]',labelpad=10, fontsize=xy_label_font_size)
        # self.axs.hold(True) 
    
    def callback_diag(self, msg):       
        print('callback data')
        #self.axes.set_autoscaley_on(True)
        self.has_new_data = True
        
        self.msg = msg.feature_selection
        # self.corners_gradient_sorted = msg.corners_gradient_sorted
        # self.grad_surfs = msg.surfs_gradient_sorted

        # self.grad_corn_mean = msg.corners_gradient_mean
        # self.grad_surf_mean = msg.surfs_gradient_mean
        # self.grad_corn_std = msg.corners_gradient_stddev
        # self.grad_surf_std = msg.surfs_gradient_stddev
        return
    
    def plot_data(self):
        if not self.has_new_data:
            return

        print('plotting')
        self.has_new_data = False

        self.axs[0, 0].clear()        
        self.axs[1, 0].clear()        
        self.axs[0, 1].clear()        
        self.axs[1, 1].clear()        
        self.axs[0, 2].clear()        
        self.axs[1, 2].clear()        
        self.axs[0, 0].set_title('Corner features')
        self.axs[1, 0].set_title('Surf features')
        self.axs[0, 1].set_title('Histogram (corners)')
        self.axs[1, 1].set_title('Histogram (surfs)')
        self.axs[0, 2].set_title('Cummulative histogram (corners)')
        self.axs[1, 2].set_title('Cummulative histogram (surfs)')

        crn_valid = [x for x in self.msg.corners_gradient_sorted if x > self.msg.corners_cutoff_thrd]
        crn_invalid = [x for x in self.msg.corners_gradient_sorted if x <= self.msg.corners_cutoff_thrd]
        srf_valid = [x for x in self.msg.surfs_gradient_sorted if x > self.msg.surfs_cutoff_thrd]
        srf_invalid = [x for x in self.msg.surfs_gradient_sorted if x <= self.msg.surfs_cutoff_thrd]

        self.axs[0, 0].hold(True)
        self.axs[0, 0].scatter(range(len(crn_valid)), crn_valid, s=5, c='red', marker='o')
        self.axs[0, 0].scatter(range(len(crn_valid), len(crn_valid) + len(crn_invalid)), crn_invalid, s=5, c='black', marker='x')
        self.axs[1, 0].hold(True)
        self.axs[1, 0].scatter(range(len(srf_valid)), srf_valid, s=5, c='red', marker='o')
        self.axs[1, 0].scatter(range(len(srf_valid), len(srf_valid) + len(srf_invalid)), srf_invalid, s=5, c='black', marker='x')

        self.axs[0, 1].hold(True)
        # n = int(self.BINS * (float(len(self.msg.corners_gradient_sorted) - len(crn_valid))) / float(len(self.msg.corners_gradient_sorted)))
        # m = int(self.BINS * (float(len(self.msg.corners_gradient_sorted) - len(crn_invalid))) / float(len(self.msg.corners_gradient_sorted)))
        # n = max(1, n)
        # m = max(1, m)
        n = self.BINS
        m = self.BINS
        _, bins_crn_valid, _ = self.axs[0, 1].hist(crn_valid, bins=n, normed=True, facecolor='red', alpha=0.75, align='left')
        _, bins_crn_invalid, _ = self.axs[0, 1].hist(crn_invalid, bins=m, normed=True, facecolor='black', alpha=0.75, align='left')

        self.axs[1, 1].hold(True)
        # n = int(self.BINS * (float(len(self.msg.surfs_gradient_sorted) - len(srf_valid))) / float(len(self.msg.surfs_gradient_sorted)))
        # m = int(self.BINS * (float(len(self.msg.surfs_gradient_sorted) - len(srf_invalid))) / float(len(self.msg.surfs_gradient_sorted)))
        # n = max(1, n)
        # m = max(1, m)
        n = self.BINS
        m = self.BINS
        _, bins_srf_valid, _ = self.axs[1, 1].hist(srf_valid, bins=n, normed=True, facecolor='red', alpha=0.75, align='left')
        _, bins_srf_invalid, _ = self.axs[1, 1].hist(srf_invalid, bins=m, normed=True, facecolor='black', alpha=0.75, align='left')
        # self.axs[0, 1].hold(True)
        # self.axs[1, 1].hold(True)
        # self.axs[0, 1].plot(crn_valid, norm.pdf(crn_valid, self.grad_corn_mean, self.grad_corn_std))
        # self.axs[1, 1].plot(bins_hist_surfs, norm.pdf(bins_hist_surfs, self.grad_surf_mean, self.grad_surf_std))
        # self.axs[0, 1].hold(False)
        # self.axs[1, 1].hold(False)

        self.axs[0, 2].hold(True)
        crn_valid_cumsum = numpy.cumsum(crn_valid)
        crn_invalid_cumsum = crn_valid_cumsum[-1] + numpy.cumsum(crn_invalid)
        self.axs[0, 2].plot(range(len(crn_valid_cumsum)), crn_valid_cumsum, c='red')
        self.axs[0, 2].plot(range(len(crn_valid_cumsum), len(crn_valid_cumsum) + len(crn_invalid_cumsum)), crn_invalid_cumsum, c='black')
        self.axs[1, 2].hold(True)
        srf_valid_cumsum = numpy.cumsum(srf_valid)
        srf_invalid_cumsum = srf_valid_cumsum[-1] + numpy.cumsum(srf_invalid)
        self.axs[1, 2].plot(range(len(srf_valid_cumsum)), srf_valid_cumsum, c='red')
        self.axs[1, 2].plot(range(len(srf_valid_cumsum), len(srf_valid_cumsum) + len(srf_invalid_cumsum)), srf_invalid_cumsum, c='black')

        plt.draw()
        plt.pause(0.1)

        return

if __name__ == "__main__":
    window = OnlineHist()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        window.plot_data()
        rate.sleep()
