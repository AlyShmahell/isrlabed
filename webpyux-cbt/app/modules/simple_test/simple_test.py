from RenderElementInterface import RenderElementInterface
import js
import configparser


class simple_test(RenderElementInterface):
    def __init__(self, data, category, type):
        self._data = data
        self._category = category
        self._type = type

    def render(self) -> None:
        config = configparser.ConfigParser()
        config.read('config.ini')

        data = self._data
        wrapper = js.document.createElement('div')
        wrapper.setAttribute('id', self._type)
        title = js.document.createElement('h3')
        title.innerText = data['label']
        wrapper.appendChild(title)

        body = js.document.createElement('div')
        body.innerHTML = data['test_custom_label_data']

        wrapper.appendChild(body)

        main = js.document.getElementById(config.get('RenderElements', 'id_result'))
        main.appendChild(wrapper)
