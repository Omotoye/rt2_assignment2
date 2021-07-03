from ipywidgets import Button, GridBox, Layout, ButtonStyle, FloatSlider

up_button = Button(
    description='',
    disabled=False,
    button_style='',  # 'success', 'info', 'warning', 'danger' or ''
    tooltip='forward',
    icon='arrow-up',  # (FontAwesome names without the `fa-` prefix)
    layout=Layout(width='auto', height='50px', grid_area='up')

)

down_button = Button(
    description='',
    disabled=False,
    button_style='',  # 'success', 'info', 'warning', 'danger' or ''
    tooltip='backward',
    icon='arrow-down',  # (FontAwesome names without the `fa-` prefix)
    layout=Layout(width='auto', height='50px', grid_area='down')
)

right_button = Button(
    description='',
    disabled=False,
    button_style='',  # 'success', 'info', 'warning', 'danger' or ''
    tooltip=' turn right',
    icon='arrow-right',  # (FontAwesome names without the `fa-` prefix)
    layout=Layout(width='auto', height='50px', grid_area='right')

)

left_button = Button(
    description='',
    disabled=False,
    button_style='',  # 'success', 'info', 'warning', 'danger' or ''
    tooltip='turn left',
    icon='arrow-left',  # (FontAwesome names without the `fa-` prefix)
    layout=Layout(width='auto', height='50px', grid_area='left')
)

stop = Button(
    description='',
    disabled=False,
    button_style='danger',  # 'success', 'info', 'warning', 'danger' or ''
    tooltip='stop',
    icon='stop',  # (FontAwesome names without the `fa-` prefix)
    layout=Layout(width='auto', height='50px', grid_area='stop')
)

start_button = Button(
    description='Start',
    disabled=False,
    button_style='',  # 'success', 'info', 'warning', 'danger' or ''
    tooltip='Start Simulation',
    icon='play',  # (FontAwesome names without the `fa-` prefix)
    layout=Layout(width='50%', height='80px')
)
stop_button = Button(
    description='Stop',
    disabled=False,
    button_style='danger',  # 'success', 'info', 'warning', 'danger' or ''
    tooltip='Stop Simulation',
    icon='stop',  # (FontAwesome names without the `fa-` prefix)
    layout=Layout(width='50%', height='80px')
)

l_velSlider = FloatSlider(
    value=1.0,
    min=0.0, max=2.0, step=0.1,
    description='Linear Velocity: ')

a_velSlider = FloatSlider(  value=1.0,
    min=0.0, max=2.0, step=0.1,
    description='Angular Velocity: ')

control_pad = GridBox(children=[up_button,down_button,left_button,right_button, stop],
        layout=Layout(
            width='100%',
            grid_template_rows='auto auto auto auto',
            grid_template_columns='16.5% 16.5% 16.5% 16.5% 16.5% 16.5%',
            grid_template_areas='''
            ". up . . . . "
            " left . right . stop ."
            " . down . . . . "
            ''')
       )