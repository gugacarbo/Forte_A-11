import { render } from 'preact'
import { App } from './app.jsx'
import './main.css'
import { AppContextProvider } from './context/data/index.jsx'

render(<AppContextProvider><App /></AppContextProvider>, document.getElementById('app'))
