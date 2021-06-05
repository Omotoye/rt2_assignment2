from ipywidgets import Button, GridBox, Layout, ButtonStyle

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
