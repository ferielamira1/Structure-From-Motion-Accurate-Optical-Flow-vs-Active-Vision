import matplotlib.pyplot as plt

def plot_figure(fig, title, filename):
    plt.figure(fig)
    plt.grid()
    plt.xlabel('Ground-truth distance to target (m)')
    plt.ylabel('Error')
    plt.title(title)
    plt.legend(loc='best')
    plt.savefig(filename)

def plot_all(gts, errors):
    """
    Plot the error and standard deviation for each type of trial, marker type, and algorithm.

    Args:
        gts (array-like): Ground truth depths.
        errors (list): List of dictionaries containing error data for each algorithm, trial type, and marker type.

    Returns:
        None
    """

    fig_gl, fig_t, fig_glvst_s, fig_glvst_o, fig_dl_s, fig_dl_o, fig_mixed_s, fig_mixed_o = range(1, 9)
    for err in errors:
        if err['trial_type'] == "translational":
            # Plotting all DL-based algorithms translation - oblique
            if err["marker_type"] == "straight":
                plt.figure(fig_dl_s)
            else:
                plt.figure(fig_dl_o)

            plt.errorbar(gts, err["error"], err["stdev"], marker='.', label=err["mname"])

        if err["mname"] == "IRR-PWC":
            # plot mixed vs pure gazelock for straight and oblique seperately
            if err['trial_type'] == 'gazelock':
                if err['marker_type'] == 'straight':
                    plt.figure(fig_mixed_s)
                else:
                    plt.figure(fig_mixed_o)

                plt.errorbar(gts, err["error_mixed"], err["stdev_mixed"], marker='.', label="mixed")
                plt.errorbar(gts, err["error"], err["stdev"], marker='.', label="{}".format("pure"))

            # plot straight vs oblique for trans and gazelock seperately
            if err['trial_type'] == 'gazelock':
                plt.figure(fig_gl)
            else:
                plt.figure(fig_t)
            plt.errorbar(gts, err["error"], err["stdev"], marker='.', label="{}".format("marker_type"))

            # Plot gazelock vs trans for straight and oblique seperately
            if err['marker_type'] == 'straight':
                plt.figure(fig_glvst_s)
            else:
                plt.figure(fig_glvst_o)
            plt.errorbar(gts, err["error"], err["stdev"], marker='.', label="{}".format(err['trial_type']))
    figures = [fig_gl, fig_t, fig_mixed_s, fig_mixed_o, fig_glvst_s,
               fig_glvst_o, fig_dl_s, fig_dl_o]
    titles = ["Error and standard deviation of \n depth estimation of a straight and oblique marker with fixation ",
              "Error and standard deviation of \n depth estimation of a straight and oblique marker \n with IRR-PWC",
              "Error and standard deviation of \n depth estimation of a straight marker with pure fixation vs fixation and\n  {}".format(
                  "IRR-PWC"),
              "Error and standard deviation of \n depth estimation of a oblique marker with pure fixation vs fixation and\n  {}".format(
                  "IRR-PWC"),
              "Error and standard deviation of \n depth estimation of a straight marker with IRR-PWC vs fixation",
              "Error and standard deviation of depth estimation \n of an oblique marker with IRR-PWC vs fixation ",
              "Error and standard deviation of depth estimation \n of a straight marker with different optical flow algorithms",
              "Error and standard deviation of depth estimation \n of a oblique marker with different optical flow algorithms"]

    for i in range(len(figures)):
        print(figures[i])
        print(titles[i])
   
        outputf="results/graph_{}.png".format(str(i))
        print(outputf)
        plot_figure(figures[i], titles[i],outputf)


