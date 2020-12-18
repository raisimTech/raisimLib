from shutil import copyfile
import datetime
import os
import ntpath

class ConfigurationSaver:
    def __init__(self, log_dir, save_items, pretrained_items=None):
        self._data_dir = log_dir + '/' + datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        os.makedirs(self._data_dir)

        if save_items is not None:
            for save_item in save_items:
                base_file_name = ntpath.basename(save_item)
                copyfile(save_item, self._data_dir + '/' + base_file_name)

        if pretrained_items is not None:
            ## pretrained_items[0] records the original file name of pretrained model; pretrained_items[1] records all the related files to the pretrained model
            pretrained_data_dir = self._data_dir + '/pretrained_' + pretrained_items[0]
            os.makedirs(pretrained_data_dir)
            for pretrained_item in pretrained_items[1]:
                base_file_name = ntpath.basename(pretrained_item)
                copyfile(save_item, pretrained_data_dir + '/' + base_file_name)

    @property
    def data_dir(self):
        return self._data_dir
        

def TensorboardLauncher(directory_path):
    from tensorboard import program
    import webbrowser
    # learning visualizer
    tb = program.TensorBoard()
    tb.configure(argv=[None, '--logdir', directory_path])
    url = tb.launch()
    print("[RAISIM_GYM] Tensorboard session created: "+url)
    webbrowser.open_new(url)
