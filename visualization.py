import fnmatch

import pandas as pd
import sys
from ggplot import *
import PyPDF2
import os


def start():
    log_dir = get_dir()
    pose_files = find_files('pose.log', log_dir)

    counter = 0
    pdfs = []
    for pose_file in pose_files:
        dir_path = os.path.dirname(pose_file)
        drone_name = os.path.relpath(dir_path, log_dir)

        out_file = str(counter) + '.pdf'
        visualize(pose_file, drone_name + ' pose', out_file)
        pdfs.append(out_file)
        counter += 1

        velocity_file = find_files('velocity.log', dir_path)[0]
        out_file = str(counter) + '.pdf'
        visualize(velocity_file, drone_name + ' velocity', out_file)
        pdfs.append(out_file)
        counter += 1

    merge_pdfs(pdfs, 'result.pdf')


def visualize(file_name, figure_name, output_file_name):
    df = pd.read_table(file_name, header=None,
                       names=['time', 'x', 'y', 'z', 'yaw', 'desired x', 'desired y', 'desired z',
                              'desired yaw'], delim_whitespace=True)

    x_plot = get_ggplot_object(df, ['x', 'desired x'], figure_name)
    y_plot = get_ggplot_object(df, ['y', 'desired y'], figure_name)
    z_plot = get_ggplot_object(df, ['z', 'desired z'], figure_name)
    yaw_plot = get_ggplot_object(df, ['yaw', 'desired yaw'], figure_name)

    generate_pdf(x_plot, y_plot, yaw_plot, z_plot, output_file_name)


def generate_pdf(x_plot, y_plot, yaw_plot, z_plot, output_file_name):
    pdfs = save_pdfs(x_plot, y_plot, z_plot, yaw_plot)

    merged_file = 'merged.pdf'
    merge_pdfs(pdfs, merged_file)

    input_file = open(merged_file, 'rb')
    input_pdf = PyPDF2.PdfFileReader(input_file)

    output_pdf = input_pdf.getPage(0)
    page1 = input_pdf.getPage(1)
    page2 = input_pdf.getPage(2)
    page3 = input_pdf.getPage(3)

    offset_x = output_pdf.mediaBox[2]
    offset_y = output_pdf.mediaBox[3]

    output_pdf.mergeTranslatedPage(page1, offset_x, 0, expand=True)
    output_pdf.mergeTranslatedPage(page2, 0, -offset_y, expand=True)
    output_pdf.mergeTranslatedPage(page3, offset_x, -offset_y, expand=True)

    output_file = open(output_file_name, 'wb')
    write_pdf = PyPDF2.PdfFileWriter()
    write_pdf.addPage(output_pdf)
    write_pdf.write(output_file)
    output_file.close()

    input_file.close()
    os.remove(merged_file)


def save_pdfs(x_plot, y_plot, z_plot, yaw_plot):
    x_plot.save('x_plot.pdf')
    y_plot.save('y_plot.pdf')
    z_plot.save('z_plot.pdf')
    yaw_plot.save('yaw_plot.pdf')
    pdfs = ['x_plot.pdf', 'y_plot.pdf', 'z_plot.pdf', 'yaw_plot.pdf']
    return pdfs


def merge_pdfs(pdfs, output_name):
    outfile = PyPDF2.PdfFileMerger()
    for f in pdfs:
        outfile.append(open(f, 'rb'))
    f = open(output_name, 'wb')
    outfile.write(f)
    f.close()
    for pdf in pdfs:
        os.remove(pdf)


def get_ggplot_object(df, value_vars, figure_name):
    df_melted = pd.melt(df, id_vars=['time'], value_vars=value_vars)
    ggplot_object = ggplot(df_melted,
                           aes(x='time', y='value', color='variable')) + geom_path() + ggtitle(
        figure_name) + ylab(value_vars[0])
    return ggplot_object


def find_files(pattern, directory):
    matches = []

    for root, dirnames, filenames in os.walk(directory):
        for filename in fnmatch.filter(filenames, pattern):
            matches.append(os.path.join(root, filename))

    return matches


def get_dir():
    try:
        config_dir = sys.argv[1]
    except IndexError:
        print('Please pass the absolute path of the configuration folder')
        exit()

    if config_dir[-1] == '/':
        config_dir = config_dir[:-1]

    if os.path.isdir(config_dir):
        return config_dir
    else:
        print(config_dir, ' is not a valid directory')
        exit()


if __name__ == '__main__':
    start()
