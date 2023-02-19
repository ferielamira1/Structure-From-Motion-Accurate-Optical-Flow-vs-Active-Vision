import matplotlib.pyplot as plt
from rosgraph_msgs.msg import std_msgs

def plot_figure(fig, title, filename):
    plt.figure(fig)
    plt.grid()
    plt.xlabel('Ground-truth distance to target (m)')
    plt.ylabel('Error')
    plt.title(title)
    plt.legend(loc='best')
    plt.savefig(filename)

def plot_gl_straight_vs_oblique(gts,errors,i):
    plt.figure(i)
    err_s=[]   
    err_o=[]

    std_s=[]
    std_o=[]
    for err in errors:
        if err["mname"] == "IRR-PWC":
            # plot mixed vs pure gazelock for straight and oblique seperately
            if err['trial_type'] == 'gazelock':
                if err['marker_type'] == 'straight':
                    err_s.append(err["error"])
                    std_s.append(err["stdev"])
                else:
                    err_o.append(err["error"])
                    std_o.append(err["stdev"])


    #plt.errorbar(gts, err["error_mixed"], err["stdev_mixed"], marker='.', label="mixed")
    plt.errorbar(gts, err_s, std_s, marker='.', label="{}".format("straight"))
    plt.errorbar(gts, err_o, std_o, marker='.', label="{}".format("oblique"))
    plt.grid()
    plt.xlabel('Ground-truth distance to target (m)')
    plt.ylabel('Error')
    plt.title("Error and standard deviation of \n depth estimation of a straight and oblique marker with fixation ")
    plt.legend(loc='best')
    plt.savefig("results/graph_{}.png".format(str(i)))

       
def plot_t_straight_vs_oblique(gts,errors,i):
    plt.figure(i)
    err_s=[]   
    err_o=[]
    std_s=[]
    std_o=[]
    for err in errors:
        if err["mname"] == "IRR-PWC":
            # plot mixed vs pure gazelock for straight and oblique seperately
            if err['trial_type'] == 'translational':
                if err['marker_type'] == 'straight':
                    err_s.append(err["error"])
                    std_s.append(err["stdev"])
                else:
                    err_o.append(err["error"])
                    std_o.append(err["stdev"])


    plt.errorbar(gts, err_s, std_s, marker='.', label="{}".format("straight"))
    plt.errorbar(gts, err_o, std_o, marker='.', label="{}".format("oblique"))

    plt.grid()
    plt.xlabel('Ground-truth distance to target (m)')
    plt.ylabel('Error')
    plt.title("Error and standard deviation of \n depth estimation of a straight and oblique marker with IRR-PWC ")
    plt.legend(loc='best')
    plt.savefig("results/graph_{}.png".format(str(i)))

       


def plot_trans_vs_fix_s(gts,errors,i):
    plt.figure(i)
    err_f=[]   
    err_t=[]
    std_f=[]
    std_t=[]
    for err in errors:
        if err["mname"] == "IRR-PWC":
            # plot mixed vs pure gazelock for straight and oblique seperately
            if err['marker_type'] == 'straight':
              if err['trial_type'] == 'gazelock':
            
                err_f.append(err["error"])
                std_f.append(err["stdev"])
              else:
                err_t.append(err["error"])
                std_t.append(err["stdev"])
                  


    #plt.errorbar(gts, err["error_mixed"], err["stdev_mixed"], marker='.', label="mixed")
    plt.errorbar(gts, err_f, std_f, marker='.', label="{}".format("fixation (pure)"))
    plt.errorbar(gts, err_t, std_t, marker='.', label="{}".format("IRR-PWC"))
    plt.grid()
    plt.xlabel('Ground-truth distance to target (m)')
    plt.ylabel('Error')
    plt.title("Error and standard deviation of \n depth estimation of an oblique marker with IRR-PWC vs fixation (pure)")
    plt.legend(loc='best')
    plt.savefig("results/graph_{}.png".format(str(i)))


def plot_trans_vs_fix_o(gts,errors,i):
    plt.figure(i)
    err_f=[]   
    err_t=[]
    std_f=[]
    std_t=[]
    for err in errors:
        if err["mname"] == "IRR-PWC":
            # plot mixed vs pure gazelock for straight and oblique seperately
            if err['marker_type'] == 'oblique':
              if err['trial_type'] == 'gazelock':
            
                err_f.append(err["error"])
                std_f.append(err["stdev"])
              else:
                err_t.append(err["error"])
                std_t.append(err["stdev"])
                  


    #plt.errorbar(gts, err["error_mixed"], err["stdev_mixed"], marker='.', label="mixed")
    plt.errorbar(gts, err_f, std_f, marker='.', label="{}".format("fixation (pure)"))
    plt.errorbar(gts, err_t, std_t, marker='.', label="{}".format("IRR-PWC"))
    plt.grid()
    plt.xlabel('Ground-truth distance to target (m)')
    plt.ylabel('Error')
    plt.title("Error and standard deviation of \n depth estimation of an oblique marker with IRR-PWC vs fixation (pure)")
    plt.legend(loc='best')
    plt.savefig("results/graph_{}.png".format(str(i)))


def plot_dl_s(gts,errors,i):
    plt.figure(i)
    n=[]
    for err in errors:
     if err["marker_type"]=="straight" and err["trial_type"]=="translational":
        if err["mname"] not in n:
          n.append(err["mname"])


    for name in n:

      errs=[]
      stdvs=[]

      for e in errors:
        if e["mname"] == name and e["trial_type"]=="translational" and e["marker_type"]=="straight":
          errs.append(e["error"])
          stdvs.append(e["stdev"])

      plt.errorbar(gts, errs, stdvs, marker='.', label=name)
      errs=[]
      stdvs=[]
    plt.grid()
    plt.xlabel('Ground-truth distance to target (m)')
    plt.ylabel('Error')
    plt.title("Error and standard deviation of \n depth estimation of a straight marker with different optical flow algorithms")
    plt.legend(loc='best')
    plt.savefig("results/graph_{}.png".format(str(i)))


def plot_dl_o(gts,errors,i):
    plt.figure(i)
    n=[]
    for err in errors:
     if err["marker_type"]=="oblique" and err["trial_type"]=="translational":
        if err["mname"] not in n:
          n.append(err["mname"])


    for name in n:

      errs=[]
      stdvs=[]

      for e in errors:
        if e["mname"] == name and e["trial_type"]=="translational" and e["marker_type"]=="oblique":
          errs.append(e["error"])
          stdvs.append(e["stdev"])

      plt.errorbar(gts, errs, stdvs, marker='.', label=name)
      errs=[]
      stdvs=[]
      
    plt.grid()
    plt.xlabel('Ground-truth distance to target (m)')
    plt.ylabel('Error')
    plt.title("Error and standard deviation of \n depth estimation of a straight marker with different optical flow algorithms")
    plt.legend(loc='best')
    plt.savefig("results/graph_{}.png".format(str(i)))

def plot_all(gts, errors):
    """
    Plot the error and standard deviation for each type of trial, marker type, and algorithm.

    Args:
        gts (array-like): Ground truth depths.
        errors (list): List of dictionaries containing error data for each algorithm, trial type, and marker type.

    Returns:
        None
    """

    counter=0
    plot_gl_straight_vs_oblique(gts,errors,counter)
    counter+=1
    plot_t_straight_vs_oblique(gts,errors,counter)
    counter+=1

    plot_dl_s(gts,errors,counter)
    counter+=1
    plot_dl_o(gts,errors,counter)
    counter+=1
    plot_trans_vs_fix_s(gts,errors,counter)
    counter+=1
    plot_trans_vs_fix_o(gts,errors,counter)

  


