from nicegui import ui 
import random


def refresh_maybe(refreshable: ui.refreshable, *args, **kwargs):
    refresh = True
    for target in refreshable.targets:
        if target.instance == refreshable.instance:
            refresh = False
    if refresh:
        refreshable(*args, **kwargs)
    else:
        refreshable.refresh(*args, **kwargs)

@ui.refreshable
def init():
    ui.label(random.random())

ui.button(on_click=lambda: refresh_maybe(init))

ui.run()