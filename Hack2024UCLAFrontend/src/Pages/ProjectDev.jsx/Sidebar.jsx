import React from 'react'
import {Link} from 'react-scroll'

function Sidebar() {
  return (
    <>
        <div className='sidebar'>
          <ul>
          <li>
              <Link
               activeClass="sidebar--active-content"
               spy={true}
               smooth={true}
               offset={-85}
               duration={500}
               to="general"
               className="sidebar--content"
              
              >
                General
              </Link>
            </li>
            <li>
              <Link
               activeClass="sidebar--active-content"
               spy={true}
               smooth={true}
               offset={-85}
               duration={500}
               to="gallery"
               className="sidebar--content"
              
              >
                Gallery
              </Link>
            </li>

            <li>
              <Link
               activeClass="sidebar--active-content"
               spy={true}
               smooth={true}
               offset={-80}
               duration={500}
               to="webSection"
               className="sidebar--content"
              
              >
                Web Design
              </Link>
            </li>
            <li>
              <Link
               activeClass="sidebar--active-content"
               spy={true}
               smooth={true}
               offset={-80}
               duration={500}
               to="circuitSection"
               className="sidebar--content"
              
              >
                  Pico&&
                  <br/>
                  circuit
              </Link>
            </li>
            <li>
              <Link
               activeClass="sidebar--active-content"
               spy={true}
               smooth={true}
               offset={-80}
               duration={500}
               to="cadSection"
               className="sidebar--content"
              
              >
                CAD
              </Link>
            </li>
          </ul>
        </div>
        
    </>
    
  )
}

export default Sidebar
